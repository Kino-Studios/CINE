# MLP correction and Depth Anything 3

## MLP correction

The trained correction weights are not distributed with this
repository, and so the correction is off by default. To use your own,
`cine_mlp_correction` reads `mlp_correction.onnx`, `mlp_correction.onnx.data` and
`scaler.json` from `cine_mlp_correction/weights/` (git ignores that folder's contents; the
build installs them to the package share). Set `mlp_weights_path` in
`cine_config/config/mlp/mlp-params.json` (or the `mlp_weights_path` launch argument) to
use another directory. Without the model the node passes trajectories through unchanged.
To run it on its own: `ros2 launch cine_mlp_correction mlp_correction.launch.py`.

## Depth Anything 3

`cine_depth` estimates depth from the camera image with
[Depth Anything 3](https://github.com/ByteDance-Seed/Depth-Anything-3) and publishes it in
metres on `/depth/image_raw`. On the drone it does not run on the Raspberry Pi: it runs on
a separate computer with an NVIDIA GTX 1660 Ti on the same ROS 2 network, which receives
the Pi's camera images and sends the depth back. Its packages are optional and are
installed into the workspace venv; Depth Anything 3 is not on PyPI, and so it is installed
from a pinned commit without its full dependency list (which pulls in 3D-export tools the
node never uses):

```bash
~/cine_ws/venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu \
  -r ~/cine_ws/src/CINE/cine_depth/requirements.txt
~/cine_ws/venv/bin/pip install --no-deps \
  "depth-anything-3 @ git+https://github.com/ByteDance-Seed/Depth-Anything-3@3d835ec1a5802d64a8b8b15f817a1ab54809bfe4"
ros2 launch cine_depth depth_publisher.launch.py
```

Drop the PyTorch CPU index on a CUDA machine. The weights download from Hugging Face on
first start. The node takes these parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `model_id` | `depth-anything/DA3METRIC-LARGE` | Hugging Face model. A `DA3METRIC` model gives depth in metres, using the focal length from `cine_config/config/camera/camera-params.json`; any other model gives relative depth, stretched over 0 to `max_depth`. |
| `process_resolution` | `504` | Longest side, in pixels, of the image the network sees. The output is resized back to the camera resolution. |
| `depth_encoding` | `32FC1` | `32FC1` publishes metres as float; `mono8` publishes depth normalised to 0 to 255. |
| `depth_scale` | `1.0` | Multiplier applied to the depth in metres. |
| `max_depth` | `10.0` | Far end, in metres, of the range relative depth is stretched over. |

The default model has 334M parameters and took about 14 s per 1152×648 frame on an
Apple M-series CPU, and so a CUDA GPU is recommended for anything close to the camera
rate. `depth-anything/DA3-SMALL` is a much lighter option, at the cost of relative depth.
DA3METRIC-LARGE, DA3-SMALL and DA3-BASE are Apache-2.0; the larger Depth Anything 3
models are CC BY-NC 4.0 (non-commercial use only).
