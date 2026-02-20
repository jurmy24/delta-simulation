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

## MJCF Model Modifications

The auto-generated `robot.xml` required several manual changes to work correctly in MuJoCo:

### Self-collision disabled

The collision mesh geoms originally had default `contype=1, conaffinity=1`, causing every robot part to collide with every other part. Because the mesh geometries overlap in the tightly packed assembly, this generated ~120 spurious contact constraints with forces up to 9 billion, completely locking the mechanism. Fixed by setting `conaffinity="0"` on the collision class so robot parts only collide with external objects (floor, grasped objects), not with each other.

### Shoulder joint ranges

The three shoulder hinge joints (`shoulder_1/2/3`) had no `range` attribute, which meant the MuJoCo viewer's Control sliders had zero range. Added `range="-1.5708 1.5708"` (±90°) to each.

### Actuator control ranges

The shoulder position actuators were missing `inheritrange="1"`, so they had no `ctrlrange` and the viewer sliders couldn't move. Added `inheritrange="1"` to all shoulder actuators.

### Actuator gain

Increased `kp` from 50 to 1000. The equality constraints that close the parallel kinematic loops are very stiff (`solref="0.005 1"`, effective stiffness ~40,000). The original `kp=50` was too weak to drive the coupled mechanism.

### Simulation options

Added `<option timestep="0.0005" solver="Newton" iterations="100" tolerance="1e-10"/>`. The Newton solver handles tightly coupled equality constraints better than the default PGS solver, and the smaller timestep prevents numerical instability with the stiff loop-closure constraints (the constraint time constant of 0.005s needs to be well above the timestep).

## Limitations

- **IK parameter mismatch**: The inverse kinematics parameters (`Fd`, `Ed`, `lower_arm`) were measured from the physical robot and may not perfectly match the simulation geometry. The square trajectory script uses the same parameters as `delta/python/delta_kinematics.py` — if the end effector path looks wrong, these need calibration against the simulation model.
- **No self-collision**: Inter-body collision is disabled for the robot. The robot can still collide with external objects (floor, etc.) but will pass through itself.
- **Soft loop closure**: The closed kinematic chains use MuJoCo equality connect constraints, which are soft (spring-damper). Under large loads or fast motions, small constraint violations (~0.01mm) are normal.
- **Gripper not tuned**: The gripper joints (`flexion_extension`, `pronation_supination`, `gripper`) have actuators but the kinematics/control for them is not implemented in the trajectory scripts.
