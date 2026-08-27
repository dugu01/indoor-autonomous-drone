#!/usr/bin/env python3
"""Resumable Stage S2.1 Python backtest runner.

Each trial runs in a fresh interpreter to avoid state leakage. Use --full-frontend
for the computationally heavier ICP/ScanContext/pose-graph path.
"""
from __future__ import annotations
import argparse, csv, json, os, subprocess, sys
from pathlib import Path

SCENARIOS = [
    "nominal","vio_outage","lidar_degraded","range_outage","baro_drift",
    "primary_imu_bias","primary_imu_freeze","backup_imu_bias",
    "primary_imu_plus_vio","all_xy_outage",
]
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--output",type=Path,default=Path("backtest_results"))
    p.add_argument("--seeds",type=int,default=3)
    p.add_argument("--scenarios",nargs="*",default=SCENARIOS)
    p.add_argument("--full-frontend",action="store_true")
    p.add_argument("--resume",action="store_true")
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=True)
    one=Path(__file__).with_name("s2_1_run_one.py")
    rows=[]
    env=os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1",MKL_NUM_THREADS="1")
    for scenario in a.scenarios:
        for seed in range(a.seeds):
            target=a.output/f"{scenario}_seed_{seed:03d}.json"
            if not (a.resume and target.exists()):
                cmd=[sys.executable,str(one),"--scenario",scenario,"--seed",str(seed),"--output",str(target)]
                if a.full_frontend: cmd.append("--full-frontend")
                subprocess.run(cmd,check=True,env=env)
            data=json.loads(target.read_text()); rows.append(data["metrics"])
            m=rows[-1]
            print(f'{scenario:23s} seed={seed:2d} max={100*m["max_error_m"]:6.2f} cm '
                  f'att={m["attitude_max_deg"]:5.2f} deg switch={m["switches"]} '
                  f'degraded={m["degraded_duration_s"]:5.2f}s pass={int(m["pass"])}')
    fields=list(rows[0])
    with (a.output/"runs.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    summary=[]
    for scenario in a.scenarios:
        g=[r for r in rows if r["scenario"]==scenario]
        summary.append(dict(
            scenario=scenario,runs=len(g),passes=sum(bool(r["pass"]) for r in g),
            worst_max_error_m=max(r["max_error_m"] for r in g),
            worst_rmse_m=max(r["rmse_m"] for r in g),
            worst_attitude_deg=max(r["attitude_max_deg"] for r in g),
            max_switch_jump_m=max(r["max_switch_jump_m"] for r in g),
            max_degraded_duration_s=max(r["degraded_duration_s"] for r in g),
            switches_total=sum(r["switches"] for r in g),
            min_lidar_acceptance=min(r["lidar_acceptance"] for r in g),
        ))
    (a.output/"summary.json").write_text(json.dumps(summary,indent=2))
    with (a.output/"summary.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
if __name__=="__main__": main()
