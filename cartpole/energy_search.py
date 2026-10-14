"""Vectorized offline energy tuning. Scalar deployment-path parity is tested.

No hardware or policy writes. The 500 Hz estimator, controller and safety equations
match their scalar counterparts; batching makes a broad candidate search practical.
"""
import math
import numpy as np

from .controllers import make
from .controllers.classic import design_gain
from .dynamics import PlantArrays, accel, rk4, wrap
from .params import H, Hardware


def batch(centre, candidates, cases, duration=12.):
    n = len(candidates)*len(cases); nc = len(cases); ticks = round(duration/H)
    defaults = make('energy',centre).p
    ps = [{**defaults, **p} for p in candidates for _ in cases]
    val = {k: np.array([p[k] for p in ps]) for k in defaults}
    truth = [c['plant'] for _ in candidates for c in cases]
    P = PlantArrays(centre, **{key: np.array([getattr(p, prop) for p in truth]) for key, prop in
         [('M_tot','M_tot'),('J','J'),('ml','ml'),('a','a'),('k_amp','k_amp'),('B_p','B_p'),('F_coul','F_coul')]},
         c_v=np.array([p.b+p.B_f for p in truth]))
    model = PlantArrays(centre); hw = Hardware()
    gain = np.repeat(np.array([design_gain(centre, dict(q_x=p['cq_x'],q_a=p['cq_a'],r=p['cr'])).ravel()
                              for p in [{**defaults,**c} for c in candidates]]),nc,axis=0)
    bias_sensor = np.array([c['bias'] for _ in candidates for c in cases])
    deadband = np.array([c['deadband'] for _ in candidates for c in cases])
    delays = np.array([p.delay_steps for p in truth]); queue = np.zeros((int(delays.max())+1,n))
    s=np.zeros((n,4)); s[:,1]=-np.pi
    hat=np.zeros_like(s); hat[:,:2]=s[:,:2]+bias_sensor
    r=np.array([hw.cart_m_per_count**2,hw.pole_rad_per_count**2])/12
    covariance=np.array([np.diag([r[j],1.]) for j in range(2)])
    F=np.array([[1.,H],[0.,1.]]); Q=np.array([2.,.3])[:,None,None]*np.array([[H**3/3,H**2/2],[H**2/2,H]])
    caught=np.zeros(n,dtype=bool); bias=np.zeros(n); previous_ad=np.zeros(n)
    fault=np.zeros(n,dtype=bool); first_fault=np.full(n,np.nan); t_up=np.full(n,np.nan)
    max_x=np.zeros(n); max_v=np.zeros(n); sum_v=np.zeros(n); sum_x=np.zeros(n); sum_a=np.zeros(n)
    tail_ok=np.ones(n,dtype=bool); tail_n=0; saturated=np.zeros(n); clamped=np.zeros(n)
    for k in range(ticks):
        x=np.rint(s[:,0]/hw.cart_m_per_count)*hw.cart_m_per_count+bias_sensor[:,0]
        a=np.rint((s[:,1]+np.pi)/hw.pole_rad_per_count)*hw.pole_rad_per_count-np.pi+bias_sensor[:,1]
        if k==0: hat[:,:2]=np.column_stack([x,a])
        for j,y in enumerate((x,a)):
            cv=F@covariance[j]@F.T+Q[j]; kg=cv[:,0]/(cv[0,0]+r[j])
            prediction=hat[:,j]+H*hat[:,j+2]; innovation=y-prediction
            hat[:,j]=prediction+kg[0]*innovation; hat[:,j+2]+=kg[1]*innovation
            covariance[j]=cv-np.outer(kg,cv[0,:])
        estimate=hat.copy(); estimate[:,1]=wrap(estimate[:,1]); x,a,xd,ad=estimate.T
        caught=np.where(caught, np.abs(a)<=np.deg2rad(val['drop_deg']),np.abs(a)<np.deg2rad(val['catch_deg']))
        E=.5*centre.J*ad**2/(centre.ml*centre.g)+np.cos(a)-1
        apex=(previous_ad*ad<0)&(np.abs(a)<np.pi/2)&(np.abs(a)>np.deg2rad(val['catch_deg']))&~caught
        bias=np.where(apex,np.minimum(.5,bias+val['adapt']*(1-np.cos(a))*.5),bias)
        previous_ad=np.where(caught,previous_ad,ad)
        w=ad*np.cos(a)
        pump=np.where((np.abs(ad)<.3)&(np.abs(a)>np.deg2rad(170))&(E< -1.9),val['kick'],val['k_e']*(val['e_over']+bias-E)*w)
        desired=np.clip(pump,-val['a_max'],val['a_max'])+(-val['k_x']*x-val['k_v']*xd)*val['eps']/(w*w+val['eps'])
        a0,_=accel(estimate,0.,model); a1,_=accel(estimate,1.,model); beta=a1-a0
        wanted=np.where(caught,-np.sum(gain*estimate,axis=1),(desired-a0)/beta)
        lo=np.maximum(-10.,(-144*(.17+x)-24*xd-a0)/beta)
        hi=np.minimum(10.,(144*(.17-x)-24*xd-a0)/beta)
        mid=np.clip((lo+hi)/2,-10,10); bad=lo>hi
        lo=np.where(bad,mid,lo); hi=np.where(bad,mid,hi)
        v=np.clip(wanted,lo,hi)
        failed=(np.abs(x)>.19)|(np.abs(ad)>60)|~np.all(np.isfinite(estimate),axis=1)
        first_fault=np.where(failed&~fault,k*H,first_fault); fault|=failed; v=np.where(fault,0.,v)
        aw=np.abs(wrap(s[:,1])); max_x=np.maximum(max_x,np.abs(s[:,0])); max_v=np.maximum(max_v,np.abs(v))
        t_up=np.where(np.isnan(t_up)&caught&(aw<np.deg2rad(10)),k*H,t_up)
        saturated+=np.abs(v)>=9.9; clamped+=np.abs(wanted-v)>.05
        if k*H >= (ticks-1)*H-2.:
            tail_n+=1; tail_ok&=aw<np.deg2rad(5); sum_v+=v*v; sum_x+=s[:,0]**2; sum_a+=aw*aw
        applied=np.sign(v)*np.maximum(0,np.abs(v)-deadband)
        queue[k%len(queue)]=applied
        applied=queue[(k-delays)%len(queue),np.arange(n)]
        next_s=rk4(rk4(s,applied,P,H/2),applied,P,H/2)
        s=np.where(fault[:,None],s,next_s)
    output=[]
    for j,params in enumerate(candidates):
        ix=slice(j*nc,(j+1)*nc); ok=(tail_ok&~fault)[ix]
        results=[]
        for i in range(j*nc,(j+1)*nc):
            results.append(dict(name=cases[i%nc]['name'],balanced_at_end=bool(tail_ok[i] and not fault[i]),
                fault=bool(fault[i]),t_upright=None if np.isnan(t_up[i]) else float(t_up[i]),
                max_abs_x_cm=float(100*max_x[i]),voltage_rms=float(np.sqrt(sum_v[i]/tail_n)),
                final_cart_rms_cm=float(100*np.sqrt(sum_x[i]/tail_n)),rms_alpha_end_deg=float(np.rad2deg(np.sqrt(sum_a[i]/tail_n))),
                clamp_fraction=float(clamped[i]/ticks),saturation_fraction=float(saturated[i]/ticks)))
        cost=1000*(nc-int(ok.sum()))+100*max_x[ix].max()+2*np.mean(np.sqrt(sum_v[ix]/tail_n))+np.mean(np.nan_to_num(t_up[ix],nan=duration))+100*np.mean(np.sqrt(sum_x[ix]/tail_n))
        output.append(dict(params=params,passed=int(ok.sum()),total=nc,score=float(cost),cases=results))
    return output
