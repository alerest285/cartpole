"""Handout Step 6 experiments, deliberately offline (no hardware entry point).

Show: smooth cart reference. ILC: repeat a trajectory with bounded learned
feedforward around LQR. MPC: sample finite-horizon plans with an LQR terminal
cost and retain the common voltage/track filter. These are coursework demos.
"""
from pathlib import Path
import json
import numpy as np

from . import lin
from .controllers import make
from .controllers.base import Controller
from .controllers.classic import design_gain
from .dynamics import linearize
from .params import H, Plant, load_plant
from .safety import SafetyConfig
from .simulate import closed_loop, summary


class TrackingILC(Controller):
    def __init__(self,p,feedforward):
        self.feedforward=feedforward
        super().__init__(p)

    def on_params(self):
        self.K=design_gain(self.plant,{})

    def step(self,s,t,ref):
        self.info['phase']='balancing'
        index=min(len(self.feedforward)-1,round(t/H))
        return float(-(self.K@(s-np.array([ref.x_ref,0,0,0])))[0]+self.feedforward[index])


class SamplingMPC(Controller):
    def on_params(self):
        A,B=linearize(self.plant); self.dt=.02
        self.A,self.B=lin.c2d(A,B,self.dt)
        self.Q=np.diag([10.,40.,0.,0.])*self.dt; self.R=.1*self.dt
        self.K,self.P=lin.dlqr(self.A,self.B,self.Q,np.array([[self.R]]))
        self.reset(None)

    def reset(self,s):
        self.k=0; self.voltage=0.; self.rng=np.random.default_rng(421)

    def step(self,s,t,ref):
        if self.k%10==0:
            x=np.tile(s-np.array([ref.x_ref,0,0,0]),(64,1)); cost=np.zeros(64)
            noise=self.rng.normal(0,.7,(30,64)); noise[:,0]=0 # feasible LQR plan included
            first=None
            for j in range(30):
                u=np.clip(-(x@self.K.ravel())+noise[j],-4,4)
                if j==0: first=u.copy()
                cost+=np.einsum('ni,ij,nj->n',x,self.Q,x)+self.R*u*u
                cost+=1e5*np.maximum(0,np.abs(x[:,0]+ref.x_ref)-.10)**2
                x=x@self.A.T+u[:,None]*self.B.ravel()
            cost+=np.einsum('ni,ij,nj->n',x,self.P,x)
            self.voltage=float(first[np.argmin(cost)])
        self.k+=1; self.info={'phase':'balancing','plans':64,'horizon_s':.6}
        return self.voltage


def run(output='docs/evidence/playground',iterations=8):
    out=Path(output); out.mkdir(parents=True,exist_ok=True); p=load_plant()
    ref=lambda t: .025*(1-np.cos(2*np.pi*t/6))/2
    report={'source':'simulation only','hardware_qualified':False}
    c=make('statefb',p)
    rows,fault=closed_loop(c,p,12,s0=[0,.03,0,0],x_ref=ref,safety=SafetyConfig(v_cap=4,x_soft=.1))
    report['show']=summary(rows,fault); np.savetxt(out/'show.csv',rows,delimiter=',')
    # A repeatable unknown Coulomb load makes feedforward learning useful.
    from dataclasses import replace
    truth=replace(p,F_coul=.25); ff=np.zeros(round(6/H)); trials=[]; best_rms=float('inf'); best_ff=ff.copy()
    for trial in range(iterations):
        ctrl=TrackingILC(p,ff)
        rows,fault=closed_loop(ctrl,truth,6,s0=[0,0,0,0],x_ref=ref,safety=SafetyConfig(v_cap=4,x_soft=.1))
        desired=np.array([ref(t) for t in rows[:,0]]); error=desired-rows[:,1]
        rms=float(np.sqrt(np.mean(error**2)))
        trials.append(dict(trial=trial,rms_cm=rms*100,**summary(rows,fault)))
        if fault or rms>best_rms*1.05:
            ff=best_ff.copy(); break
        best_rms=rms; best_ff=ff.copy()
        # Kx < 0: positive command offset produces negative equilibrium x.
        correction=-3*np.convolve(error,np.ones(51)/51,mode='same')
        ff=np.clip(.995*ff+correction,-1,1)
        np.savetxt(out/f'ilc-{trial:02d}.csv',np.column_stack([rows,desired,error]),delimiter=',')
    report['ilc']=trials; np.save(out/'ilc-feedforward.npy',best_ff)
    rows,fault=closed_loop(SamplingMPC(p),p,8,s0=[0,.05,0,0],pokes=[(3,.5)],
                           safety=SafetyConfig(v_cap=4,x_soft=.1))
    report['mpc']=summary(rows,fault); np.savetxt(out/'mpc.csv',rows,delimiter=',')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2)); return report
