#!/usr/bin/env python3
"""Convert a validated student TorchScript export to a static single-input CPU ONNX."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch
import onnx
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"src/m20_heightmap"))
from m20_heightmap.contract import contract, validate_model_contract


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--contract", type=Path, required=True)
    ap.add_argument("--profile", choices=("v0", "v1"), required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise ValueError("Use a new ONNX output path; do not overwrite an existing release")
    actual = json.loads(args.contract.read_text())
    spec = contract(args.profile, actual.get("history_steps", 1))
    validate_model_contract(args.contract, spec)
    torch.set_num_threads(1)
    actor = torch.jit.load(str(args.model), map_location="cpu").eval()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sample = torch.zeros(1, spec["input_dim"])
    torch.onnx.export(actor, sample, str(args.output), input_names=["obs"], output_names=["actions"],
                      opset_version=17, dynamo=False, do_constant_folding=True)
    graph = onnx.load(str(args.output))
    onnx.checker.check_model(graph)
    options = ort.SessionOptions()
    options.intra_op_num_threads, options.inter_op_num_threads = 1, 1
    session = ort.InferenceSession(str(args.output), sess_options=options, providers=["CPUExecutionProvider"])
    if len(session.get_inputs()) != 1 or session.get_inputs()[0].shape != [1, spec["input_dim"]]:
        raise ValueError("ONNX input must be static [1,student_dim]")
    if len(session.get_outputs()) != 1 or session.get_outputs()[0].shape != [1, 16]:
        raise ValueError("ONNX output must be static [1,16]")
    rng = np.random.default_rng(42)
    worst = 0.
    for _ in range(32):
        x = rng.normal(size=(1, spec["input_dim"])).astype(np.float32)
        with torch.inference_mode():
            expected = actor(torch.from_numpy(x)).numpy()
        observed = session.run(["actions"], {"obs": x})[0]
        if not np.isfinite(observed).all():
            raise ValueError("ONNX returns nonfinite actions")
        np.testing.assert_allclose(observed, expected, atol=2e-5, rtol=2e-5)
        worst = max(worst, float(np.max(np.abs(expected-observed))))
    record = {"profile": args.profile, "input_shape": [1, spec["input_dim"]], "output_shape": [1, 16],
              "input_name": "obs", "output_name": "actions", "opset": 17, "providers": session.get_providers(),
              "policy_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
              "torchscript_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
              "contract_sha256": hashlib.sha256(args.contract.read_bytes()).hexdigest(),
              "parity_samples": 32, "max_abs_error": worst, "scope": "CPU numerical export parity, not robot closed-loop"}
    args.output.with_suffix(".onnx.json").write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
