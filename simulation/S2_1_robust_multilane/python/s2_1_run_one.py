#!/usr/bin/env python3
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import s2_1_reference as s
p=argparse.ArgumentParser();p.add_argument('--scenario',required=True);p.add_argument('--seed',type=int,default=0);p.add_argument('--output',type=Path,required=True);p.add_argument('--full-frontend',action='store_true');p.add_argument('--history',action='store_true');a=p.parse_args()
r=s.run_trial(a.seed,s.scenario_catalog()[a.scenario],s.Config(),full_frontend=a.full_frontend,save_history=a.history)
a.output.parent.mkdir(parents=True,exist_ok=True)
h=r.pop('history',None)
if h is not None:
 import numpy as np
 np.savez_compressed(a.output.with_suffix('.npz'),**h)
with a.output.open('w') as f:json.dump(r,f,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else x)
print(json.dumps(r['metrics']))
