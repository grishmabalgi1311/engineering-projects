"""Experimental recovery around the preserved hybrid RL policy."""
from hybrid_rl_policy import MyAgent as WorkingHybrid
from oscillation_recovery import OscillationMonitor

class MyAgent(WorkingHybrid):
    def setup(self,info):
        super().setup(info)
        self.oscillation_monitor=OscillationMonitor()
        self.oscillation_recoveries=0
    def act(self,obs):
        result=super().act(obs)
        eligible=(self.phase=='route' and not self.local_final
                  and self.local_parent_until is None and self.index<self.end-12)
        if not eligible:
            self.oscillation_monitor.reset()
            return result
        if self.oscillation_monitor.observe(obs.step,self.index):
            self.oscillation_recoveries+=1
            self.oscillation_monitor.recovered(obs.step)
            return self.parent_mode(obs,self.index+1,'repeated translation reversals')
        action=result[0] if isinstance(result,tuple) else result
        self.oscillation_monitor.record(obs.step,action)
        return result
    def finish(self,result):
        print('OSCILLATION recoveries',self.oscillation_recoveries,flush=True)
        return super().finish(result)
