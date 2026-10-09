#!/usr/bin/env python3
"""Writes a stub ONNX 'detector' that ignores its input and always outputs ONE detection (class 0 'stub_object', YOLOv8 layout) at the centre
of the 640 x 640 letterboxed input. For checking the detector node's plumbing (TF, floor geometry, messages) on live camera frames without a
real model:    python3 make_stub_onnx.py /home/dev/models/stub.onnx
"""
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper


def main(path, nc=2, n=100):
    out = np.zeros((1, 4 + nc, n), np.float32)
    out[0, :4, 0] = [320.0, 320.0, 80.0, 60.0]          # cx, cy, w, h in the letterboxed input
    out[0, 4, 0] = 0.9                                   # class 0
    const = helper.make_node("Constant", [], ["output0"], value=helper.make_tensor("v", TensorProto.FLOAT, out.shape, out.flatten().tolist()))
    graph = helper.make_graph([const], "stub", [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 640, 640])],
                              [helper.make_tensor_value_info("output0", TensorProto.FLOAT, list(out.shape))])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 7
    meta = model.metadata_props.add()
    meta.key, meta.value = "names", "{0: 'stub_object', 1: 'other'}"
    onnx.save(model, path)
    print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1])
