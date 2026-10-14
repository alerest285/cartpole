"""Executable evidence for handout modeling, linear design and the Q experiment."""
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from cartpole import lin
from cartpole.controllers import make
from cartpole.controllers.classic import design_gain
from cartpole.dynamics import linearize
from cartpole.params import H,load_plant
from cartpole.safety import SafetyConfig
from cartpole.simulate import closed_loop,summary


def prepare():
    out=ROOT/'docs/evidence/course'; out.mkdir(parents=True,exist_ok=True)
    p=load_plant(); A,B=linearize(p); C=np.array([[1,0,0,0],[0,1,0,0]]); D=np.zeros((2,1))
    Ad,Bd=lin.c2d(A,B,H)
    report={'A':A.tolist(),'B':B.tolist(),'C':C.tolist(),'D':D.tolist(),
        'controllability_rank':int(np.linalg.matrix_rank(lin.ctrb(A,B))),
        'observability_rank':int(np.linalg.matrix_rank(lin.obsv(A,C))),
        'sample_period_s':H,'source':'simulation only','plant':p.to_dict(),'Q_experiment':[]}
    for name,true in [('full_motor',p),('ideal_force_gain',replace(p,K_m=0.,rotor_mass=False,B_p=0.,F_coul=0.))]:
        rows,fault=closed_loop(make('sine_test',p),true,5)
        np.savetxt(out/f'sine_5V_10Hz_{name}.csv',rows,delimiter=',',
                   header='t,x,alpha,xd,ad,xd_hat,ad_hat,v_des,v,balancing',comments='')
        report[name]=summary(rows,fault)
    for q in ([10,40,0,0],[40,40,0,0],[10,160,0,0],[10,40,1,1]):
        values=dict(zip(['q_x','q_a','q_xd','q_ad'],q)); c=make('statefb',p); c.set_params(values)
        K=design_gain(p,values); Kc=design_gain(p,{**values,'time':'continuous'})
        rows,fault=closed_loop(c,p,8,s0=[0,.05,0,0],pokes=[(3,.6)],safety=SafetyConfig(v_cap=4))
        report['Q_experiment'].append({'Q':q,'R':.1,'K_discrete':K.ravel().tolist(),
            'K_continuous':Kc.ravel().tolist(),'closed_loop_poles_z':np.abs(np.linalg.eigvals(Ad-Bd@K)).tolist(),
            **summary(rows,fault)})
    K=design_gain(p,{'method':'place','time':'continuous'})
    report['pole_placement']={'requested':[-5,-6,-7,-8],'K':K.ravel().tolist(),
                              'achieved':sorted(np.real(np.linalg.eigvals(A-B@K)).tolist())}
    (out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Saved course analysis, Q comparison and 5 V / 10 Hz traces:',out)
    return report


if __name__=='__main__': prepare()
