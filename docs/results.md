# Results

From the paper. Unless marked otherwise, these come from 15 trials in kinematic
simulation (the drone model follows the planned path exactly; there is no aerodynamics).

| What was measured | Without | With |
| --- | --- | --- |
| Peak speed while filming | 3.0 m/s | 1.2 m/s |
| Time the subject stayed within 0.1 m of the focus distance | 23.6% | 76.4% |
| Largest deviation from the focus distance | 0.72 m | 0.31 m |
| Position error (root mean square), without and with the drift correction | 0.25 m | 0.05 m |
| Path points within 10 cm of their target, without and with the drift correction | about 25% | over 95% |
| Camera tilt when a bright light enters the view | 0° | 20°, with the heading turned about 110° away |

A bench test on the real camera, with the drone held and moved by hand instead of flying,
gave one hardware result: the subject's average distance from its rule-of-thirds target
fell from 127.6 px to 32.5 px (75% lower), and the variance of that distance fell by 98%,
over 33 frames without CINE and 66 with it.

## Limitations

Every flight result is from simulation. The depth model gives relative
depth, not metres, unless a metric model is used. Cost weights were tuned by hand. CINE
frames one subject at a time. The drift correction was trained on simulated drift, so it
has not been shown to help with the drift the real sensors produce.

The next step is to fix the real drone's position drift (the
[open issue in CINE-Sensing](https://github.com/Kino-Studios/CINE-Sensing/blob/main/docs/vio-testing.md#open-issue-vio-drift))
and then fly the same baseline-against-CINE comparison outdoors.
