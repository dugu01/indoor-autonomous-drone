from __future__ import annotations
from pathlib import Path
import hashlib,json,py_compile,shutil,time

def sha256(path:Path)->str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def ensure_project_validation_compat(project:Path,package_root:Path,out:Path)->dict:
    """Install only validation-layer v6 migrations; never touch autonomy/frozen parent."""
    project=project.resolve();package_root=package_root.resolve();out=out.resolve();srcdir=package_root/'resources'/'project_validation_compat';vdir=project/'coupled'/'validation'
    bdir=out/'project_compat_backup';bdir.mkdir(parents=True,exist_ok=True);actions=[];stamp=time.strftime('%Y%m%d_%H%M%S')
    targets=[
        ('audit_S2_4_E_static.py',lambda t:r'(?:literal|adversarial)-competing-corridors-candidate' not in t),
        ('audit_S2_4_E_cumulative_overlay_portable.py',lambda t:r'(?:literal|adversarial)-competing-corridors-candidate' not in t),
        ('audit_S2_4_E_literal_corridor_geometry.py',lambda t:'def fmt_metric' not in t),
        ('test_S2_4_E_competing_decision_contract.m',lambda t:'S2_4_E_LAYERED_V6' not in t),
        ('validate_S2_4_E_milestone_2.m',lambda t:'S2_4_E_LAYERED_V6' not in t),
        ('validate_S2_4_E_competing_corridors_multiseed.m',lambda t:'S2_4_E_LAYERED_V6' not in t),
    ]
    for name,needs in targets:
        dst=vdir/name;src=srcdir/name
        if not dst.exists() or not src.exists():actions.append({'file':name,'status':'missing_not_patched','dst_exists':dst.exists(),'src_exists':src.exists()});continue
        text=dst.read_text(encoding='utf-8',errors='replace')
        if not needs(text):actions.append({'file':name,'status':'already_compatible','sha256':sha256(dst)});continue
        backup=bdir/f'{dst.stem}_{stamp}{dst.suffix}';shutil.copy2(dst,backup);shutil.copy2(src,dst)
        if dst.suffix=='.py':py_compile.compile(str(dst),doraise=True)
        actions.append({'file':name,'status':'patched','backup':str(backup),'sha256':sha256(dst)})
    report={'schema':'S2_4_PROJECT_VALIDATION_COMPAT_V6','actions':actions};(out/'project_compat_report.json').write_text(json.dumps(report,indent=2)+'\n');return report
