#!/usr/bin/env python3
"""Operator-independent offline validation: never loads ROS or an SDK interface."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def main():
    actual=json.loads((ROOT/'models/v1/observation_contract.json').read_text())
    if actual.get('deployment_family')=='m20_cts_student':
        subprocess.run([sys.executable,str(ROOT/'scripts/offline_cts.py'),
                        '--probe',str(ROOT/'install/m20_student_sdk/lib/m20_student_sdk/student_onnx_probe')],check=True)
        return
    probe=ROOT/'install/m20_student_sdk/lib/m20_student_sdk/student_onnx_probe'
    if not probe.is_file():raise RuntimeError('Build m20_student_sdk on AOS first')
    reports=[]
    for profile in ('v0','v1'):
        fixtures=ROOT/'evidence/offline_models'/profile
        with tempfile.TemporaryDirectory(prefix='m20_offline_') as tmp:
            out=Path(tmp)/'actions.f32'
            p=subprocess.run([str(probe),str(ROOT/'models'/profile/'policy.onnx'),str(fixtures/'input.f32'),str(out)],capture_output=True,text=True,timeout=30,check=True)
            expected=np.fromfile(str(fixtures/'expected.f32'),dtype='<f4').reshape(-1,16)
            actual=np.fromfile(str(out),dtype='<f4').reshape(-1,16)
            np.testing.assert_allclose(actual,expected,atol=2e-5,rtol=2e-5)
            reports.append({'profile':profile,'frames':len(actual),'max_absolute_error':float(np.max(np.abs(actual-expected))),'probe':json.loads(p.stdout),'hardware_commands':0})
    print(json.dumps({'status':'passed','scope':'AOS CPU ONNX offline; not policy control startup','models':reports},indent=2))

if __name__=='__main__':main()
