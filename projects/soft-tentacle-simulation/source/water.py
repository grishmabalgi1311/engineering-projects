"""Linear depth-averaged free-surface surrogate on a staggered reflecting grid.

eta_t = -H div(u), u_t = -g grad(eta) - damping*u + force/(rho H area).
No external sinusoid, direct goal force, or independent ball steering is used.
This is a long-wave approximation, not 3D CFD or complete Morison inertia.
"""
import numpy as np


class Water:
    def __init__(self,cfg):
        self.L=cfg['tank_length_m']; self.W=cfg['tank_width_m']
        self.nx=cfg['grid_nx']; self.ny=cfg['grid_ny']
        self.H=cfg['water_depth_m']; self.rho=cfg['water_density_kg_m3']
        self.damping=cfg['wave_damping_per_s']
        self.dx=self.L/self.nx; self.dy=self.W/self.ny
        self.x=(np.arange(self.nx)+.5)*self.dx-self.L/2
        self.y=(np.arange(self.ny)+.5)*self.dy-self.W/2
        self.reset()

    def reset(self):
        self.eta=np.zeros((self.ny,self.nx))
        self.u=np.zeros((self.ny,self.nx+1))
        self.v=np.zeros((self.ny+1,self.nx))

    def weights(self,xy):
        xy=np.atleast_2d(xy)
        gx=np.clip((xy[:,0]+self.L/2)/self.dx-.5,0,self.nx-1-1e-10)
        gy=np.clip((xy[:,1]+self.W/2)/self.dy-.5,0,self.ny-1-1e-10)
        ix=gx.astype(int); iy=gy.astype(int)
        fx=gx-ix; fy=gy-iy
        return ix,iy,fx,fy

    def interpolate(self,a,weights):
        x,y,fx,fy=weights
        return ((1-fx)*(1-fy)*a[y,x]+fx*(1-fy)*a[y,x+1]
                +(1-fx)*fy*a[y+1,x]+fx*fy*a[y+1,x+1])

    def sample(self,xy):
        weights=self.weights(xy)
        uc=.5*(self.u[:,:-1]+self.u[:,1:])
        vc=.5*(self.v[:-1,:]+self.v[1:,:])
        sy,sx=np.gradient(self.eta,self.dy,self.dx)
        return (np.column_stack([self.interpolate(uc,weights),self.interpolate(vc,weights)]),
                self.interpolate(self.eta,weights),
                np.column_stack([self.interpolate(sx,weights),self.interpolate(sy,weights)]))

    def step(self,dt,positions=None,forces=None):
        cfl=dt*np.sqrt(9.81*self.H)*np.sqrt(1/self.dx**2+1/self.dy**2)
        if cfl >= .8:
            raise ValueError(f'Water timestep violates conservative CFL limit: {cfl:.3f}')
        fx=np.zeros_like(self.eta); fy=np.zeros_like(self.eta)
        if positions is not None and len(positions):
            ix,iy,wx,wy=self.weights(positions)
            for ox,oy,w in ((0,0,(1-wx)*(1-wy)),(1,0,wx*(1-wy)),
                            (0,1,(1-wx)*wy),(1,1,wx*wy)):
                np.add.at(fx,(iy+oy,ix+ox),forces[:,0]*w)
                np.add.at(fy,(iy+oy,ix+ox),forces[:,1]*w)
        scale=1/(self.rho*self.H*self.dx*self.dy)
        self.u[:,1:-1] += dt*(-9.81*np.diff(self.eta,axis=1)/self.dx
                              +.5*(fx[:,:-1]+fx[:,1:])*scale)
        self.v[1:-1,:] += dt*(-9.81*np.diff(self.eta,axis=0)/self.dy
                              +.5*(fy[:-1,:]+fy[1:,:])*scale)
        self.u *= np.exp(-self.damping*dt)
        self.v *= np.exp(-self.damping*dt)
        # Closed walls: zero normal flow. No absorbing edge is silently added.
        self.u[:,[0,-1]]=0; self.v[[0,-1],:]=0
        self.eta -= dt*self.H*(np.diff(self.u,axis=1)/self.dx+np.diff(self.v,axis=0)/self.dy)
        if not np.isfinite(self.eta).all() or np.max(np.abs(self.eta)) > .25*self.H:
            raise RuntimeError('Water left small-amplitude model range; reduce actuation or refine model')

    def energy(self):
        return float(self.rho*self.dx*self.dy*(.5*9.81*np.sum(self.eta**2)
                     +.5*self.H*(np.sum(self.u**2)+np.sum(self.v**2))))
