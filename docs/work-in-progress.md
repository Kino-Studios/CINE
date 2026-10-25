# Work in progress

Early work for later stages. It stays in the tree but is not started by
`cine_bringup`.

- `cine_agent/cine_agent/setpoint_adjustment/nodes/orbit_waypoint_generator_node.py`:
  generates orbit waypoints around the subject from visual-inertial pose and focus depth.
- `cine_agent/cine_agent/setpoint_sender/trapezoidal_setpoint_filter.py`: trapezoidal
  velocity profile for position setpoints, the start of a trapezoidal motion controller.
- `cine_agent/cine_agent/light_vector/vector_analysis.py`: estimates the light direction
  from the camera image.
- `cine_agent/launch/collect_baseline_data.launch.py`,
  `cine_agent/launch/collect_cinematic_data.launch.py`,
  `cine_agent/launch/collect_sim_data.launch.py`: launch-file versions of the data
  collection scripts.
- `cine_depth/launch/depth_publisher.launch.py`: starts the depth publisher, which no
  pipeline uses yet.
