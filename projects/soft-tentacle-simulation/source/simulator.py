"""Two-way horizontal drag coupling to a reduced water grid and buoyant ball."""
import json
from pathlib import Path
import numpy as np
import mujoco
from build_model import build, ROOT
from water import Water

BOUNDS=np.array([0,15,40.5,64,85,104.5,122,138,153,165.5,178.5,
                 189.5,200,209,217.5,225.5,232.5,239,249.68])*1e-3
WIDTH=np.array([50.574,46.149,42.110,38.425,35.063,31.994,29.195,26.640,
                24.309,22.182,20.240,18.469,16.853,15.378,14.033,12.805,11.684,10.662])*1e-3
HEIGHT=np.array([13,23.5,20.46,19.5,17.68,15.85,14.78,12.52,11.55,10.65,
                 10.07,8.23,7.26,6.45,5.61,4.63,4.2,3.9])*1e-3


def sphere_volume(radius,height):
    h=np.clip(height,0,2*radius)
    return np.pi*h*h*(radius-h/3)


class Simulation:
    def __init__(self,config_path=None,overrides=None):
        self.cfg=json.loads(Path(config_path or ROOT/'config.json').read_text())
        self.cfg.update(overrides or {})
        c=self.cfg
        if not all(np.isfinite(c[k]) and c[k]>0 for k in (
            'tank_length_m','tank_width_m','water_depth_m','dt_s','control_dt_s',
            'water_density_kg_m3','arm_mass_kg','ball_radius_m','ball_mass_kg','max_tension_N')):
            raise ValueError('Geometry, time steps, density, masses and tension limit must be positive')
        if min(c['grid_nx'],c['grid_ny'])<4:
            raise ValueError('Need at least four grid cells per direction')
        if c['ball_mass_kg']>=c['water_density_kg_m3']*sphere_volume(c['ball_radius_m'],2*c['ball_radius_m']):
            raise ValueError('Configured ball cannot float')
        for name in ('wave_damping_per_s','arm_cd_lateral','arm_cd_axial','ball_cd',
                     'ball_vertical_damping_Ns_m'):
            if not np.isfinite(c[name]) or c[name]<0:
                raise ValueError(f'{name} must be finite and nonnegative')
        ratio=c['control_dt_s']/c['dt_s']
        if abs(ratio-round(ratio))>1e-8:
            raise ValueError('control_dt_s must be an integer multiple of dt_s')
        self.substeps=round(ratio)
        tree=build(c)
        tree.getroot().find('compiler').set('meshdir',str(ROOT/'geometry/meshes'))
        import xml.etree.ElementTree as ET
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        self.water=Water(c)
        self.arm_ids=np.array([self.model.body(f'arm{a}_seg{k}').id
                               for a in (1,) for k in range(1,19)])
        self.ball_id=self.model.body('ball').id
        self.ball_geom=self.model.geom('ball_geom').id
        self.ball_q=int(self.model.jnt_qposadr[self.model.joint('ball_free').id])
        self.arm_geoms={self.model.geom(f'arm{a}_g{k}').id for a in (1,) for k in range(1,19)}
        self.tendon_ids=[self.model.tendon(f'arm{a}_cable{k}').id for a in (1,) for k in (1,2)]
        self.drag_enabled=True
        self.hold_time=0
        self.last_reaction=np.zeros(2)
        self.reset()

    def reset(self,seed=None,ball_xy=None,target_xy=None):
        rng=np.random.default_rng(seed)
        mujoco.mj_resetData(self.model,self.data)
        self.water.reset()
        c=self.cfg
        self.target=np.asarray(target_xy if target_xy is not None else c['target_xy_m'],dtype=float)
        xy=np.asarray(ball_xy if ball_xy is not None else c.get('ball_start_xy_m',[0,-.02]),dtype=float)
        for point in (xy,self.target):
            if point.shape!=(2,) or not np.isfinite(point).all() or np.any(
                np.abs(point)>np.array([c['tank_length_m'],c['tank_width_m']])/2-c['ball_radius_m']):
                raise ValueError('Ball/target must lie inside tank with radius clearance')
        lo,hi=0,2*c['ball_radius_m']
        for _ in range(60):
            mid=(lo+hi)/2
            if sphere_volume(c['ball_radius_m'],mid)*c['water_density_kg_m3']<c['ball_mass_kg']:
                lo=mid
            else:
                hi=mid
        self.data.qpos[self.ball_q:self.ball_q+3]=[*xy,c['surface_z_m']+c['ball_radius_m']-(lo+hi)/2]
        self.model.site_pos[self.model.site('target').id,:2]=self.target
        self.data.ctrl[:]=0
        self.hold_time=0
        self.steps=0
        self.contacted=False
        self.drag_enabled=True
        mujoco.mj_forward(self.model,self.data)
        self.initial_lengths=self.data.ten_length[self.tendon_ids].copy()
        frame=self.frame()
        self.history=np.tile(frame,4)
        return self.history.copy(),self.info()

    def velocity(self,bid):
        vel=np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,bid,vel,0)
        return vel[3:]

    def body_forces(self):
        c=self.cfg; d=self.data
        points=np.vstack([d.xipos[self.arm_ids,:2],d.xipos[self.ball_id,:2]])
        uv,eta,slopes=self.water.sample(points)
        reaction=np.zeros((len(points),2))
        spans=np.diff(BOUNDS)
        for n,bid in enumerate(self.arm_ids):
            k=n%18
            z=d.xipos[bid,2]
            waterline=c['surface_z_m']+eta[n]
            s=np.clip((waterline-z)/(HEIGHT[k]/2),-1,1)
            fraction=.5+(np.arcsin(s)+s*np.sqrt(1-s*s))/np.pi
            volume=np.pi/4*WIDTH[k]*HEIGHT[k]*spans[k]*fraction
            R=d.xmat[bid].reshape(3,3)
            axis=R[:2,0]
            rel=self.velocity(bid)[:2]-uv[n]
            axial=np.dot(rel,axis)*axis
            lateral=rel-axial
            wet_height=np.clip(waterline-(z-HEIGHT[k]/2),0,HEIGHT[k])
            flat_area=spans[k]*wet_height
            end_area=np.pi/4*WIDTH[k]*HEIGHT[k]*fraction
            force=-.5*c['water_density_kg_m3']*(
                c['arm_cd_lateral']*flat_area*np.linalg.norm(lateral)*lateral+
                c['arm_cd_axial']*end_area*np.linalg.norm(axial)*axial)
            if not self.drag_enabled:
                force[:]=0
            d.xfrc_applied[bid,:2]+=force
            d.xfrc_applied[bid,2]+=c['water_density_kg_m3']*9.81*volume
            reaction[n]=-force
        # Hydrostatic buoyancy of a spherical cap with a reduced horizontal
        # pressure-slope term. Exact wave scattering and radiation are not resolved.
        bid=self.ball_id; n=len(points)-1
        vel=self.velocity(bid)
        r=c['ball_radius_m']
        h=np.clip(c['surface_z_m']+eta[n]-d.xipos[bid,2]+r,0,2*r)
        vol=sphere_volume(r,h)
        rel=vel[:2]-uv[n]
        drag=-.5*c['water_density_kg_m3']*c['ball_cd']*np.pi*r*r*(h/(2*r))*np.linalg.norm(rel)*rel
        slope_force=-c['water_density_kg_m3']*9.81*vol*slopes[n]
        force=drag+slope_force if self.drag_enabled else np.zeros(2)
        d.xfrc_applied[bid,:2]+=force
        d.xfrc_applied[bid,2]+=c['water_density_kg_m3']*9.81*vol-c['ball_vertical_damping_Ns_m']*vel[2]
        reaction[n]=-force
        self.last_reaction=reaction.sum(axis=0)
        return points,reaction

    def tick(self,tensions,perturb=None,external_force=None):
        target=np.asarray(tensions,dtype=float)
        if target.shape!=(2,) or not np.isfinite(target).all():
            raise ValueError('Two finite tendon commands required')
        if np.any(target<0) or np.any(target>self.cfg['max_tension_N']):
            raise ValueError('Tension outside configured simulation range')
        self.data.ctrl[:]=target
        mujoco.mj_forward(self.model,self.data)
        self.data.xfrc_applied[:]=0
        if perturb is not None:
            mujoco.mjv_applyPerturbForce(self.model,self.data,perturb)
        if external_force is not None:
            bid, force = external_force
            self.data.xfrc_applied[bid,:3] += force
        points,reaction=self.body_forces()
        self.water.step(self.cfg['dt_s'],points,reaction)
        mujoco.mj_step(self.model,self.data)
        mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or np.any(self.data.warning.number):
            raise RuntimeError('MuJoCo numerical warning or nonfinite state')
        for contact in self.data.contact:
            pair={int(contact.geom1),int(contact.geom2)}
            if self.ball_geom in pair and pair.intersection(self.arm_geoms):
                self.contacted=True
        self.steps+=1

    def frame(self):
        # Sparse, potentially camera-measurable shape positions; no fluid-grid
        # state or target-derived forces are exposed as privileged actor inputs.
        marker_ids=[self.model.body(f'arm{a}_seg{k}').id for a in (1,) for k in (5,9,14,18)]
        return np.r_[self.data.xipos[self.ball_id,:2],self.velocity(self.ball_id)[:2],
                     self.target,self.data.ctrl,self.data.xpos[marker_ids,:2].ravel()]

    def info(self):
        distance=float(np.linalg.norm(self.data.xipos[self.ball_id,:2]-self.target))
        return dict(time_s=float(self.data.time),distance_m=distance,
                    water_energy_J=self.water.energy(),wave_peak_m=float(np.max(np.abs(self.water.eta))),
                    contact=self.contacted,success=self.hold_time>=1.,hold_time_s=self.hold_time,
                    ball_xy=self.data.xipos[self.ball_id,:2].copy())

    def step(self,action):
        action=np.asarray(action,dtype=float)
        if action.shape!=(2,) or not np.isfinite(action).all() or np.any(np.abs(action)>1):
            raise ValueError('Action must be two finite values in [-1,1]')
        tension=(action+1)*.5*self.cfg['max_tension_N']
        old=self.data.ctrl.copy()
        for _ in range(self.substeps):
            self.tick(tension)
        self.history=np.r_[self.history[len(self.frame()):],self.frame()]
        info=self.info()
        if info['distance_m']<=self.cfg['goal_radius_m'] and not info['contact']:
            self.hold_time+=self.cfg['control_dt_s']
        else:
            self.hold_time=0
        info=self.info()
        reward=(-info['distance_m']-0.002*float(np.sum(tension**2))
                -0.002*float(np.sum((tension-old)**2))-5*int(info['contact'])
                +int(info['success']))
        terminated=bool(info['contact'] or info['success'])
        truncated=bool(self.data.time>=self.cfg['episode_s'])
        return self.history.copy().astype(np.float32),reward,terminated,truncated,info

    def save_state(self,path):
        np.savez_compressed(path,time=self.data.time,qpos=self.data.qpos,qvel=self.data.qvel,
                            ctrl=self.data.ctrl,eta=self.water.eta,u=self.water.u,v=self.water.v,
                            target=self.target,config_json=json.dumps(self.cfg))


def demo_tensions(t,limit=2):
    # Different phases create a repeatable experiment, not a trained controller.
    a=.6*np.sin(2*np.pi*.6*t)
    b=.6*np.sin(2*np.pi*.6*t+np.pi/2)
    return np.clip([.6+a,.6-a],0,limit)
