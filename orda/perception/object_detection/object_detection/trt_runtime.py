"""TensorRT 검출기 백엔드.

왜 ONNX Runtime 이 아니라 TensorRT 인가
---------------------------------------
이 보드(Jetson Orin, sm_87)에서 YOLO 검출기를 GPU 로 올리는 길은 TensorRT 뿐이다.

  * PyPI 의 onnxruntime-gpu 1.29.0 은 aarch64/cp312 휠이 있고 CUDA 13 + cuDNN 9
    빌드라 설치 자체는 깨끗하게 된다. 그런데 실제로 돌리면 첫 노드에서
    `CUDA error cudaErrorNoKernelImageForDevice` 로 죽는다 — 범용 휠에 sm_87
    커널이 안 들어 있다(Jetson 전용 아키텍처라 보통 빠진다).
  * Jetson AI Lab 인덱스에도 onnxruntime-gpu 는 jp6/cu126/cp310 까지만 있다.
    이 보드는 JP7 / CUDA 13 / Python 3.12 라 받을 수 있는 휠이 없다.
  * TensorRT 는 엔진을 **이 장비에서 빌드**하므로 sm_87 커널이 그때 생긴다.

실측(train10_detector_best.onnx, 640x640, 실제 카메라 20 프레임)
    ONNX Runtime CPU      560.4 ms
    TensorRT fp16          16.6 ms   (34x)
    검출 결과              20/20 완전 일치

엔진 파일
--------
엔진은 GPU 아키텍처와 TensorRT 버전에 묶인다. 다른 장비나 다른 TRT 로 만든
엔진을 그대로 읽으면 조용히 깨지므로, 파일 이름에 둘 다 박고 로드 전에
검증한다. 빌드는 `scripts/build_trt_engine.py` 가 한다(약 7~8분).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

DEFAULT_CACHE_DIR = Path.home() / ".cache" / "xycar_trt"


def engine_filename(onnx_path: str) -> str:
    """이 장비/이 TensorRT 에서만 유효한 엔진 파일 이름을 만든다.

    이름에 sm 과 TRT 버전을 박아 두면, 보드를 바꾸거나 TensorRT 를 올린 뒤에
    옛 엔진을 잘못 집어 드는 일이 생기지 않는다 — 파일 이름부터 안 맞는다.
    """
    import tensorrt as trt
    import torch

    stem = Path(onnx_path).stem
    major, minor = torch.cuda.get_device_capability(0)
    return f"{stem}.sm{major}{minor}.trt{trt.__version__}.fp16.engine"


def default_engine_path(onnx_path: str) -> Path:
    return DEFAULT_CACHE_DIR / engine_filename(onnx_path)


class TensorRTDetector:
    """단일 입력/단일 출력 검출 엔진 실행기.

    ONNX Runtime 세션의 자리에 그대로 들어가도록 run(blob) -> np.ndarray 하나만
    제공한다. 호출자는 어느 백엔드인지 몰라도 된다.
    """

    def __init__(self, engine_path: str):
        import tensorrt as trt
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; TensorRT backend needs a GPU")

        self._torch = torch
        self._logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(self._logger)
        with open(engine_path, "rb") as handle:
            engine = runtime.deserialize_cuda_engine(handle.read())
        if engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine: {engine_path}")
        self._engine = engine
        self._context = engine.create_execution_context()

        names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
        inputs = [n for n in names if engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        outputs = [n for n in names if engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError(
                f"expected 1 input and 1 output, got {inputs} / {outputs}"
            )
        self._in_name, self._out_name = inputs[0], outputs[0]
        self.input_shape = tuple(engine.get_tensor_shape(self._in_name))
        self.output_shape = tuple(engine.get_tensor_shape(self._out_name))

        # 버퍼는 한 번만 잡고 매 프레임 재사용한다. 프레임마다 새로 할당하면
        # cudaMalloc 이 동기화 지점을 만들어 지연이 튄다.
        self._d_in = torch.empty(self.input_shape, dtype=torch.float32, device="cuda")
        self._d_out = torch.empty(self.output_shape, dtype=torch.float32, device="cuda")
        # 호스트->디바이스 복사는 pinned 메모리에서 훨씬 빠르다.
        self._h_in = torch.empty(self.input_shape, dtype=torch.float32).pin_memory()
        self._stream = torch.cuda.Stream()

        self._context.set_input_shape(self._in_name, self.input_shape)
        self._context.set_tensor_address(self._in_name, self._d_in.data_ptr())
        self._context.set_tensor_address(self._out_name, self._d_out.data_ptr())

        self.engine_path = str(engine_path)

    def run(self, blob: np.ndarray) -> np.ndarray:
        """ORT 의 session.run(None, {name: blob})[0] 과 같은 것을 돌려준다."""
        torch = self._torch
        if tuple(blob.shape) != self.input_shape:
            raise ValueError(
                f"blob shape {tuple(blob.shape)} != engine input {self.input_shape}"
            )
        self._h_in.copy_(torch.from_numpy(np.ascontiguousarray(blob, dtype=np.float32)))
        with torch.cuda.stream(self._stream):
            self._d_in.copy_(self._h_in, non_blocking=True)
            self._context.execute_async_v3(self._stream.cuda_stream)
        self._stream.synchronize()
        return self._d_out.cpu().numpy()

    def warmup(self, runs: int = 3) -> None:
        blob = np.zeros(self.input_shape, dtype=np.float32)
        for _ in range(runs):
            self.run(blob)


def try_load(engine_path: Optional[str], onnx_path: str):
    """(detector, 설명문자열) 을 돌려준다. 못 쓰면 (None, 이유).

    엔진이 없거나 안 맞는 것은 정상적인 상황이다 — 처음 돌리는 장비이거나
    TensorRT 를 올린 직후다. 그때는 호출자가 ONNX Runtime CPU 로 내려간다.
    """
    path = Path(engine_path) if engine_path else default_engine_path(onnx_path)
    if not path.is_file():
        return None, f"engine not found: {path}"
    try:
        detector = TensorRTDetector(str(path))
    except Exception as exc:  # noqa: BLE001 - 어떤 실패든 CPU 로 내려가면 된다
        return None, f"engine load failed ({path}): {exc}"
    return detector, f"TensorRT engine {path}"
