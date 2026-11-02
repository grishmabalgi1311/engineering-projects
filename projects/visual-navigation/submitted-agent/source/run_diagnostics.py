"""Non-controlling diagnostics for public observations and policy decisions."""
import contextlib
import csv
import dataclasses
import hashlib
import io
import json
import sys
import uuid
from collections import deque
from pathlib import Path
import cv2
from vis_nav_sdk.agent import decide


class StepLog(io.TextIOBase):
    def __init__(self,raw,console):
        self.raw=raw;self.console=console;self.step=0;self.context='';self.pending='';self.last={}

    def write(self,text):
        self.raw.write(text)
        self.pending+=text
        while '\n' in self.pending:
            line,self.pending=self.pending.split('\n',1)
            prefix=next((p for p in ('pose unavailable ','pose ','visual approach ') if line.startswith(p)),None)
            key=(prefix,self.context)
            if prefix is None or key not in self.last or self.step-self.last[key]>=25:
                self.console.write(line+'\n');self.last[key]=self.step
        return len(text)

    def flush(self):
        if not self.raw.closed:self.raw.flush()
        if not self.console.closed:self.console.flush()


def instrument(controller):
    class DiagnosticAgent(controller):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.diagnostic_folder=None;self.diagnostic_stream=None;self.diagnostic_csv=None
            self.diagnostic_raw=None;self.diagnostic_last_image=-25;self.diagnostic_actions=deque()
            self.diagnostic_warning=-100

        def setup(self,info):
            self._close_diagnostics()
            self.diagnostic_folder=None;self.diagnostic_stream=None
            self.diagnostic_actions.clear();self.diagnostic_last_image=-25;self.diagnostic_warning=-100
            if info.session_id=='preflight':
                print('SDK preflight uses synthetic targets; the real target setup follows.',flush=True)
                return super().setup(info)
            try:
                folder=self.target_cache_path.parent/'diagnostics'/f'{info.session_id}_{uuid.uuid4().hex[:8]}'
                folder.mkdir(parents=True)
                for i,target in enumerate(info.targets):
                    if not cv2.imwrite(str(folder/f'target_{i}.png'),target):
                        raise OSError('Could not save diagnostic target image')
                source=Path(__file__).parent
                (folder/'metadata.json').write_text(json.dumps(dict(
                    controller=f'{controller.__module__}.{controller.__qualname__}',session_id=info.session_id,
                    camera_K=info.camera.intrinsic_matrix.tolist(),
                    source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*.py')},
                    inputs='Public RGB, SDK targets, K, decisions and inferred localization only. No simulator truth.'),indent=2))
                self.diagnostic_raw=(folder/'full_policy.log').open('w',encoding='utf-8',buffering=1)
                self.diagnostic_stream=StepLog(self.diagnostic_raw,sys.stdout)
                self.diagnostic_csv=(folder/'decisions.csv').open('w',newline='',encoding='utf-8')
                self.diagnostic_writer=csv.writer(self.diagnostic_csv)
                self.diagnostic_writer.writerow(['step','phase','index','anchor','current_node','goal_node',
                    'inferred_x','inferred_z','inferred_yaw','local_lost','predicted_age','action','repeat','turn_reversals',
                    'parent_x','parent_z','parent_yaw','parent_inliers','failed_pose'])
                self.diagnostic_folder=folder
            except (OSError,cv2.error) as error:
                self._close_diagnostics();print('Diagnostics unavailable:',error,flush=True)
            if self.diagnostic_stream is None:
                return super().setup(info)
            print('Controller:',controller.__module__,'diagnostics:',self.diagnostic_folder,flush=True)
            with contextlib.redirect_stdout(self.diagnostic_stream):
                super().setup(info)

        def act(self,obs):
            if self.diagnostic_stream is None:
                return super().act(obs)
            stream=self.diagnostic_stream;stream.step=obs.step
            stream.context=(getattr(self,'phase',None),getattr(self,'index',None))
            with contextlib.redirect_stdout(stream):
                result=super().act(obs)
            action,repeat=decide(result,1)
            turn=(bool(action&8)-bool(action&4))*(-1 if action&2 else 1)
            if turn:self.diagnostic_actions.append((obs.step,int(turn),getattr(self,'index',None)))
            while self.diagnostic_actions and self.diagnostic_actions[0][0]<obs.step-30:
                self.diagnostic_actions.popleft()
            signs=[s for _,s,i in self.diagnostic_actions if i==getattr(self,'index',None)]
            reversals=sum(a!=b for a,b in zip(signs,signs[1:]))
            try:
                pose=getattr(self,'local_pose',None)
                values=list(pose[:3]) if pose is not None else [None]*3
                self.diagnostic_writer.writerow([obs.step,getattr(self,'phase',None),getattr(self,'index',None),
                    getattr(self,'local_anchor',None),getattr(self,'current',None),getattr(self,'goal',None),
                    *values,getattr(self,'local_lost',None),getattr(self,'predicted_age',None),action,repeat,reversals,
                    *(list(self.predicted) if getattr(self,'predicted',None) is not None else [None]*4),
                    getattr(self,'failed_pose',None)])
                self.diagnostic_csv.flush()
                if obs.step-self.diagnostic_last_image>=25:
                    cv2.imwrite(str(self.diagnostic_folder/f'frame_{obs.step:06d}.jpg'),obs.image)
                    self.diagnostic_last_image=obs.step
                if reversals>=4 and obs.step-self.diagnostic_warning>=50:
                    print('DIAGNOSTIC repeated steering reversals at step',obs.step,
                          'waypoint',getattr(self,'index',None),'count',reversals,flush=True)
                    self.diagnostic_warning=obs.step
            except (OSError,cv2.error) as error:
                print('Diagnostics stopped:',error,flush=True);self._close_diagnostics()
            return result

        def finish(self,result):
            if self.diagnostic_folder is not None:
                try:
                    value=dataclasses.asdict(result) if result is not None else None
                    (self.diagnostic_folder/'result.json').write_text(json.dumps(value,indent=2))
                except OSError as error:print('Could not save diagnostic result:',error,flush=True)
            self._close_diagnostics()
            return super().finish(result)

        def _close_diagnostics(self):
            for name in ('diagnostic_csv','diagnostic_raw'):
                handle=getattr(self,name,None)
                if handle is not None:
                    try:handle.close()
                    except OSError:pass
                    setattr(self,name,None)
            self.diagnostic_stream=None
    return DiagnosticAgent
