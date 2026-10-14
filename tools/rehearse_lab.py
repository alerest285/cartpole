"""Synthetic first-session rehearsal: record -> identify -> refit -> re-test.

Runs and fits live in a temporary directory. Exports evidence labelled simulated;
never opens hardware, changes the current fit, or promotes a policy.
"""
import json
import os
import shutil
import sys
import tempfile
from dataclasses import replace
from pathlib import Path

ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))


def main():
    out=ROOT/'docs/evidence/lab-rehearsal'; out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='cartpole-rehearsal-') as scratch:
        os.environ['CARTPOLE_DATA']=str(Path(scratch)/'data')
        os.environ['CARTPOLE_RUNS']=str(Path(scratch)/'runs')
        import numpy as np
        from cartpole.params import Plant,load_plant
        from cartpole.backends.sim import SimBackend
        from cartpole.runtime import Runtime
        from cartpole import sysid
        from cartpole.friction import estimate
        from cartpole.simulate import run
        true=replace(Plant(),B_f=3.3,k_amp=.93,B_p=.004,F_coul=.15,delay_steps=1,m_c=.60)
        rt=Runtime(SimBackend(true,realtime=False),plant=Plant(),log=lambda _:None)
        rt.set_mode('excite'); rt.set_params('excite',{'amp':1.5,'noise':.5})
        rt.set_safety({'v_cap':4.}); rt.arm()
        for _ in range(20000):
            rt.backend.wait_tick(); rt._tick()
        data=np.array(rt.run_rows)
        assert rt.armed,rt.fault
        rt.disarm(); rt.stop()
        fit=sysid.fit(include_sim=True)
        assert fit is not None,'No identification result'
        accepted=sysid.save(fit)
        assert accepted,fit['problems']
        hw=rt.backend.hw
        columns={'t':data[:,0],'x':data[:,7]*hw.cart_m_per_count,
                 'alpha':data[:,8]*hw.pole_rad_per_count-np.pi,'v':data[:,6]}
        np.savetxt(out/'simulated-regression.csv',np.column_stack(list(columns.values())),
                   delimiter=',',header=','.join(columns),comments='')
        scalar=estimate(columns,plant=true,delay_steps=1)
        before=run('energy',Plant(),true,12)[1]
        after=run('energy',load_plant(),true,12)[1]
        report={'source':'simulation only','true_plant':true.to_dict(),'fit':fit,
                'scalar_friction_with_known_constants':scalar,'before_refit':before,'after_refit':after,
                'fit_installed_only_in_temporary_directory':True}
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__': main()
