"""ONNX Runtime session factory: the CUDA GPU when it is usable, the CPU otherwise.

The container's python is 3.8 (no recent torch), so the GPU path is onnxruntime-gpu 1.19 plus the pip CUDA 12 / cuDNN 9 libraries, installed in their
own folder (YOLO_GPULIBS, default /home/dev/pylibs_gpu: `pip install --target ... onnxruntime-gpu==1.19.2 nvidia-cudnn-cu12 nvidia-cublas-cu12
nvidia-cuda-runtime-cu12 nvidia-cufft-cu12 nvidia-curand-cu12 nvidia-cuda-nvrtc-cu12 nvidia-nvjitlink-cu12 --no-deps`). That folder is put first on
sys.path (its onnxruntime shadows the CPU one) and its shared libraries are preloaded, since LD_LIBRARY_PATH cannot be changed after start.
Import this module BEFORE anything imports onnxruntime. On the RTX 5070 (sm_120) the kernels are JIT-compiled from PTX: the first run of a process
takes about 40 s (the driver caches the result for the later ones), about 35 ms per 640 px frame afterwards.
"""
import ctypes
import glob
import os
import sys

GPULIBS = os.environ.get("YOLO_GPULIBS", "/home/dev/pylibs_gpu")
_LIB_ORDER = ("cuda_runtime", "nvjitlink", "cublas", "cufft", "curand", "cuda_nvrtc", "cudnn")


def _enable_gpu_libs(folder=None):
    """Puts the GPU folder first on sys.path and preloads its CUDA libraries. True if the folder exists."""
    folder = folder or GPULIBS
    if not os.path.isdir(os.path.join(folder, "onnxruntime")):
        return False
    if folder not in sys.path:
        sys.path.insert(0, folder)
    for sub in _LIB_ORDER:
        for lib in sorted(glob.glob(os.path.join(folder, "nvidia", sub, "lib", "*.so*"))):
            try:
                ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass                                                  # optional library, the provider reports what is really missing
    return True


def make_session(model, threads=8, device="auto"):
    """(onnxruntime.InferenceSession, providers actually used). device: auto (GPU if possible) | gpu (same, but fails if CUDA is missing) | cpu."""
    if device not in ("auto", "gpu", "cpu"):
        raise ValueError("device must be auto, gpu or cpu, not %r" % (device,))
    if device != "cpu":
        _enable_gpu_libs()
    import onnxruntime as ort
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = int(threads)
    want_gpu = device != "cpu" and "CUDAExecutionProvider" in ort.get_available_providers()
    if device == "gpu" and not want_gpu:
        raise RuntimeError("device 'gpu' requested but the CUDA execution provider is not available (is %s installed?)" % GPULIBS)
    session = ort.InferenceSession(model, opts, providers=(["CUDAExecutionProvider"] if want_gpu else []) + ["CPUExecutionProvider"])
    used = session.get_providers()
    if device == "gpu" and "CUDAExecutionProvider" not in used:
        raise RuntimeError("device 'gpu' requested but the session fell back to %s (CUDA libraries not loadable)" % used)
    return session, used
