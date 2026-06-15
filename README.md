# pairs_uav_trackers

Reference-tracking plugins for the PAIRS multirotor-UAV autonomy stack. A tracker turns
high-level commands (a goto goal, a landing, a flip) into the smooth, dynamically feasible
full-state reference that a controller then follows. The trackers are loaded at runtime by
the `pairs_uav_managers` control manager through pluginlib, so the active tracker can be
switched in flight.

## Contents

Pluginlib trackers built against the `pairs_uav_managers` `Tracker` interface:

- `MpcTracker` — model-predictive trajectory tracker for smooth, constrained motion (the main flight tracker)
- `LineTracker` — straight-line point-to-point motion
- `LandoffTracker` — landing and takeoff sequences
- `MidairActivationTracker` — engages control when activated mid-air
- `FlipTracker` — aggressive flip maneuver

The MPC tracker links the prebuilt `MpcTrackerSolver` under `lib/` (arm64 and x64 builds).
A ready-to-run flip-tracker demo lives under `tmux/flip_tracker/`.

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-pairs-uav-trackers
```

## Usage

Run the flip-tracker demo (simulation):

```bash
cd tmux/flip_tracker && ./start.sh
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_trackers` package; the original
copyright is retained in [LICENSE](LICENSE).
