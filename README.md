# Delta Robot Simulation

MuJoCo simulation of the FR8 delta robot. The MJCF model (`simulation/delta-robot/robot.xml`) was auto-generated from the Onshape CAD assembly using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot).

[Onshape source](https://cad.onshape.com/documents/20bfbde5f0dfed09746f3835/w/89e87a07589996b404d06798/e/02e6df3ca634eef0ea4fbf46)

## Setup

Requires Python 3.11+ and [uv](https://docs.astral.sh/uv/).

```bash
uv sync
```

## Running

**Interactive viewer** (use the Control panel to move joints):

```bash
uv run python simulation/run_sim.py delta-robot
```

**Square trajectory demo** (end effector traces a square):

```bash
uv run python scripts/square_trajectory.py
```

## Configuration

All MuJoCo simulation settings are configured in `simulation/delta-robot/config.json` and automatically applied when running `onshape-to-robot`. No manual XML editing is required.

## Limitations

- **IK parameter mismatch**: The inverse kinematics parameters (`Fd`, `Ed`, `lower_arm`) were measured from the physical robot and may not perfectly match the simulation geometry. The square trajectory script uses the same parameters as `delta/python/delta_kinematics.py` — if the end effector path looks wrong, these need calibration against the simulation model.
- **No self-collision**: Inter-body collision is disabled for the robot. The robot can still collide with external objects (floor, etc.) but will pass through itself.
- **Soft loop closure**: The closed kinematic chains use MuJoCo equality connect constraints, which are soft (spring-damper). Under large loads or fast motions, small constraint violations (~0.01mm) are normal.
- **Gripper not tuned**: The gripper joints (`flexion_extension`, `pronation_supination`, `gripper`) have actuators but the kinematics/control for them is not implemented in the trajectory scripts.
