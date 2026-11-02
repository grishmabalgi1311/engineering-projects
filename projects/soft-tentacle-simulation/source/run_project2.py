"""Run the uncalibrated Project 2 reduced water/ball prototype."""
import argparse
import csv
from pathlib import Path
import time
import numpy as np
import mujoco
from simulator import Simulation, demo_tensions, ROOT


def draw_water(scene,sim):
    """Draw cell patches at actual eta (no visual height amplification)."""
    w=sim.water
    for j in range(w.ny):
        for i in range(w.nx):
            if scene.ngeom>=scene.maxgeom:
                return
            g=scene.geoms[scene.ngeom]
            height=float(w.eta[j,i])
            color=np.clip(.5+height/.00008,0,1)
            mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_BOX,
                               np.array([w.dx*.46,w.dy*.46,.00015]),
                               np.array([w.x[i],w.y[j],sim.cfg['surface_z_m']+height]),
                               np.eye(3).ravel(),np.array([.04+.1*color,.2+.55*color,.55+.4*color,.65],dtype=np.float32))
            scene.ngeom+=1

    # Floating square markers visualize the simulated horizontal velocity.
    now = sim.data.time
    if not hasattr(sim, '_pixels') or now < sim._pixel_time:
        rng = np.random.default_rng(3)
        sim._pixels = rng.uniform([-.48*w.L,-.48*w.W],[.48*w.L,.48*w.W],(160,2))
        sim._pixel_time = now
    elapsed = min(.1, max(0., now-sim._pixel_time))
    uv, _, _ = w.sample(sim._pixels)
    sim._pixels += elapsed*uv
    sim._pixels = np.clip(sim._pixels,[-.49*w.L,-.49*w.W],[.49*w.L,.49*w.W])
    sim._pixel_time = now
    _, heights, _ = w.sample(sim._pixels)
    for xy,z in zip(sim._pixels,heights):
        if scene.ngeom >= scene.maxgeom: break
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_BOX,
            np.array([.0014,.0014,.0003]),np.array([*xy,sim.cfg['surface_z_m']+z+.001]),
            np.eye(3).ravel(),np.array([.6,.95,1.,.9],dtype=np.float32))
        scene.ngeom += 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(ROOT/'config.json'))
    p.add_argument('--headless',action='store_true')
    p.add_argument('--duration',type=float,default=3600)
    p.add_argument('--mode',choices=['demo','manual','rest'],default='manual')
    p.add_argument('--log',default=str(ROOT/'results/project2.csv'))
    p.add_argument('--snapshot',help='Final full-state .npz file')
    p.add_argument('--render',help='Final offscreen PNG (needs Pillow)')
    args=p.parse_args()
    if not np.isfinite(args.duration) or args.duration<=0:
        p.error('Duration must be finite and positive')
    sim=Simulation(args.config)
    state={'paused':False,'demo':args.mode=='demo','reset':False}
    events=[]
    def key(code):
        events.append(code)

    def handle_events(viewer):
        while events:
            code=events.pop(0)
            if code==32:
                state['paused']=not state['paused']
            elif code in (ord('B'),ord('b')):
                # New episode: timestamp resets; CSV episode column increments.
                state['reset']=True
                viewer.perturb.active=0
            elif code in (ord('M'),ord('m')):
                state['demo']=not state['demo']
                print('Oscillation demo:',state['demo'])
            elif code in (ord('U'),ord('u'),ord('C'),ord('c')):
                sim.data.ctrl[:]=0
                state['demo']=False
                print('Two tendon commands cleared; water keeps moving and decays.')
            elif code in (ord('V'),ord('v')):
                sim.drag_enabled=not sim.drag_enabled
                print('Horizontal water/body coupling:',sim.drag_enabled)
            else:
                for ch,index,delta in [('I',0,.1),('K',0,-.1),('O',1,.1),('L',1,-.1),
                                        ]:
                    if code in (ord(ch),ord(ch.lower())):
                        state['demo']=False
                        sim.data.ctrl[index]=np.clip(sim.data.ctrl[index]+delta,0,sim.cfg['max_tension_N'])
                        print('Tensions [N]:',sim.data.ctrl)

    path=Path(args.log)
    path.parent.mkdir(parents=True,exist_ok=True)
    print('SINGLE FIN PIXEL TANK: UNCALIBRATED REDUCED WATER MODEL; not 3D CFD or trained RL.')
    print('Demo oscillates tendons; no prescribed current or target-seeking force.')
    print('M demo/manual; I/K O/L arm1; U/C release; B reset; Space pause; V coupling')
    print('Select segment by double-click, then Ctrl + right-drag. Physical motion remains planar.')
    print('CSV:',path)
    episode=0
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['episode','time_s',*[f'tension{i}_N' for i in range(1,3)],
                         'ball_x_m','ball_y_m','ball_z_m','ball_vx_ms','ball_vy_ms',
                         'target_x_m','target_y_m','distance_m','wave_peak_m','water_energy_J',
                         'ball_arm_contact','horizontal_coupling_on',*[f'cable{i}_mm' for i in range(1,3)]])
        def log():
            info=sim.info()
            writer.writerow([episode,sim.data.time,*sim.data.ctrl,*sim.data.xipos[sim.ball_id],
                             *sim.velocity(sim.ball_id)[:2],*sim.target,info['distance_m'],
                             info['wave_peak_m'],info['water_energy_J'],int(info['contact']),
                             int(sim.drag_enabled),*(sim.data.ten_length[sim.tendon_ids]*1000)])
            f.flush()

        def advance(perturb=None):
            if state['demo']:
                sim.data.ctrl[:]=demo_tensions(sim.data.time,sim.cfg['max_tension_N'])
            # Mirror viewer sliders each step; Clear all therefore also works in manual mode.
            sim.tick(sim.data.ctrl.copy(),perturb)
            if sim.steps%10==0:
                log()
            if sim.steps%500==0:
                print(f"t={sim.data.time:.1f}s ball={sim.info()['ball_xy']} peak={sim.info()['wave_peak_m']*1000:.3f} mm")

        log()
        if args.headless:
            while sim.data.time<args.duration-1e-10:
                advance()
        else:
            from mujoco import viewer as mj_viewer
            with mj_viewer.launch_passive(sim.model,sim.data,key_callback=key) as viewer:
                viewer.cam.lookat[:]=[0,0,0]
                viewer.cam.distance=.72
                viewer.cam.azimuth=100
                viewer.cam.elevation=-20
                while viewer.is_running() and sim.data.time<args.duration-1e-10:
                    start=time.perf_counter()
                    with viewer.lock():
                        handle_events(viewer)
                        if state['reset']:
                            sim.reset()
                            episode+=1
                            log()
                            state['reset']=False
                        if not state['paused']:
                            advance(viewer.perturb)
                        viewer.user_scn.ngeom=0
                        draw_water(viewer.user_scn,sim)
                    viewer.sync()
                    time.sleep(max(0,sim.cfg['dt_s']-(time.perf_counter()-start)))
        log()
    if args.snapshot:
        sim.save_state(args.snapshot)
    if args.render:
        from PIL import Image
        with mujoco.Renderer(sim.model,height=800,width=1200) as renderer:
            camera=mujoco.MjvCamera()
            camera.lookat[:]=[0,0,0]
            camera.distance=.72; camera.azimuth=100; camera.elevation=-20
            renderer.update_scene(sim.data,camera=camera)
            draw_water(renderer.scene,sim)
            Image.fromarray(renderer.render()).save(args.render)
    print('Completed:',sim.info())


if __name__=='__main__':
    main()
