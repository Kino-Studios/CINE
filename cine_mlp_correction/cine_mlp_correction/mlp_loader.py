import json
import logging
import os

import numpy as np
import onnxruntime as ort

logger = logging.getLogger(__name__)


class MLPLoader:
    def __init__(self, weights_dir: str) -> None:
        self._session = None
        self._input_name = None
        self._scaler_mean = None
        self._scaler_std = None

        onnx_path = os.path.join(weights_dir, 'mlp_correction.onnx')
        scaler_path = os.path.join(weights_dir, 'scaler.json')

        if not os.path.isfile(onnx_path):
            logger.warning('ONNX model not found at %s, switching to passthrough', onnx_path)

            return

        with open(scaler_path) as f:
            scaler = json.load(f)

        self._scaler_mean = np.array(scaler['mean'], dtype=np.float32)
        self._scaler_std = np.array(scaler['std'], dtype=np.float32)

        self._session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        self._input_name = self._session.get_inputs()[0].name
        logger.info('Loaded model from %s', onnx_path)

    def correct(self, pos_pts: np.ndarray, dt: float) -> np.ndarray:
        if self._session is None:
            return pos_pts

        pos_pts = np.asarray(pos_pts, dtype=np.float32)
        n = len(pos_pts)

        vel = np.empty_like(pos_pts)
        if n == 1:
            vel[0] = 0.0
        else:
            vel[0] = (pos_pts[1] - pos_pts[0]) / dt
            vel[-1] = (pos_pts[-1] - pos_pts[-2]) / dt
            if n > 2:
                vel[1:-1] = (pos_pts[2:] - pos_pts[:-2]) / (2.0 * dt)

        x = np.concatenate([pos_pts, vel], axis=1)
        x = (x - self._scaler_mean) / self._scaler_std

        delta = self._session.run(None, {self._input_name: x})[0]

        return pos_pts + delta.astype(np.float32)
