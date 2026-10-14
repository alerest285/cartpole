"""Bounded offline search; simulation proposals only, never installed automatically."""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from cartpole.energy_search import batch
from cartpole.robustness import scenarios, evaluate
from cartpole.params import load_plant


def search(count=128, seconds=600):
    rng=np.random.default_rng(714); p=load_plant(); start=time.monotonic()
    train=scenarios(p,6,17)+scenarios(p,12,91)[1:]
    candidates=[{},dict(adapt=.3),dict(e_over=.3,adapt=.3)]
    for _ in range(count):
        candidates.append(dict(k_e=float(rng.uniform(2,10)),a_max=float(rng.uniform(3.5,9)),
            k_x=float(rng.uniform(.2,7)),k_v=float(rng.uniform(.3,4)),eps=float(rng.uniform(.3,8)),
            kick=float(rng.uniform(1,4)),e_over=float(rng.uniform(0,.5)),adapt=float(rng.uniform(0,1)),
            catch_deg=float(rng.uniform(25,40)),cq_x=float(rng.uniform(1,8)),
            cq_a=float(rng.uniform(70,180)),cr=float(rng.uniform(.03,.12))))
    out=ROOT/'docs/evidence/energy-search-batched.json'; trials=[]
    for first in range(0,len(candidates),32):
        if time.monotonic()-start>seconds: break
        trials.extend(batch(p,candidates[first:first+32],train,12))
        best=min(trials,key=lambda r:r['score'])
        out.write_text(json.dumps({'training_seeds':[17,91],'trials':trials,'best':best},indent=2)+'\n')
        print(f"{len(trials)} candidates; best {best['passed']}/{best['total']}; {time.monotonic()-start:.1f} s",flush=True)
    best=min(trials,key=lambda r:r['score'])
    # Fresh seed, not inspected while designing/searching candidates.
    held=evaluate('energy',best['params'],p,scenarios(p,24,991),10,12)
    (ROOT/'docs/evidence/energy-search-final-validation.json').write_text(json.dumps(held,indent=2)+'\n')
    print('Final unseen-set result',held['passed'],'/',held['total'],flush=True)
    print('Candidate parameters',json.dumps(best['params']),flush=True)
    print('No runtime/controller defaults changed.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--candidates',type=int,default=128)
    parser.add_argument('--seconds',type=int,default=600); args=parser.parse_args()
    search(args.candidates,args.seconds)
