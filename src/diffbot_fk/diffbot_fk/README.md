# Python modules

- `kinematics.py`          : forward/inverse kinematics equations + `DiffDriveOdometry` class (no ROS, tested in `test/`).
- `fk_odometry_node.py`    : `/joint_states` -> `/fk/odom`, `/fk/distance`, `/fk/path`, TF odom->base_footprint, service `/fk/reset`.
- `keypad_teleop_node.py`  : keyboard/numpad -> `/cmd_vel`.
- `patterns.py`            : segment planner (square) closed on the FK pose, no ROS (tested in `test/`).
- `pattern_driver_node.py` : `/fk/odom` -> `/cmd_vel`, drives the square and logs each segment.
