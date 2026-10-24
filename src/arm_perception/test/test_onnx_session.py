#!/usr/bin/env python3
"""Unit tests of onnx_session.make_session: the CPU path, the fallback and the strict gpu mode (no GPU needed)."""
import importlib.util
import os
import tempfile
import unittest

from arm_perception import onnx_session as osn

_HERE = os.path.dirname(os.path.abspath(__file__))


def stub_model(path):
    spec = importlib.util.spec_from_file_location("make_stub_onnx", os.path.join(_HERE, "make_stub_onnx.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.main(path)


class MakeSessionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp()
        cls.model = os.path.join(cls.dir, "stub.onnx")
        stub_model(cls.model)

    def test_cpu(self):
        session, used = osn.make_session(self.model, 2, "cpu")
        self.assertEqual(used, ["CPUExecutionProvider"])
        self.assertEqual(session.get_inputs()[0].shape[2], 640)

    def test_auto_always_yields_a_working_session(self):
        session, used = osn.make_session(self.model, 2, "auto")
        self.assertIn("CPUExecutionProvider", used)

    def test_gpu_mode_is_strict(self):
        """'gpu' must fail loudly rather than silently run on the CPU."""
        try:
            _, used = osn.make_session(self.model, 2, "gpu")
        except RuntimeError as e:
            self.assertIn("CUDA", str(e))
        else:
            self.assertEqual(used[0], "CUDAExecutionProvider")

    def test_bad_device(self):
        with self.assertRaises(ValueError):
            osn.make_session(self.model, 2, "tpu")

    def test_missing_gpu_folder_is_not_an_error(self):
        self.assertFalse(osn._enable_gpu_libs(os.path.join(self.dir, "nope")))


if __name__ == "__main__":
    unittest.main()
