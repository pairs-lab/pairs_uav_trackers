# pairs_uav_trackers

Reference-tracking plugins for the PAIRS multirotor-UAV autonomy stack. A tracker turns
high-level commands (a goto goal, a velocity request, a landing, a manual joystick input)
into the smooth, dynamically feasible full-state reference that a controller then follows.
The trackers are loaded at runtime by the `pairs_uav_managers` control manager through
pluginlib, so the active tracker can be switched in flight.

## Contents

Pluginlib trackers built against the `pairs_uav_managers` `Tracker` interface:

- `MpcTracker` — model-predictive trajectory tracker for smooth, constrained motion (the main flight tracker)
- `LineTracker` — straight-line point-to-point motion
- `LandoffTracker` — landing and takeoff sequences
- `JoyTracker` — manual joystick control
- `SpeedTracker` — follows external velocity/speed references
- `FlipTracker` — aggressive flip maneuver
- `MidairActivationTracker` — engages control when activated mid-air

The MPC tracker links the prebuilt `MpcTrackerSolver` under `lib/`.

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)

```bash
sudo apt install ros-noetic-pairs-uav-trackers
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_trackers` package; the original
copyright is retained in [LICENSE](LICENSE).
