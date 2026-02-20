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

Most settings are now configured in `config.json` and will survive re-exports. However, after re-running `onshape-to-robot`, you must manually apply these changes to `robot.xml`:

### Base plate height

After re-export, change the `base_plate` body position from `pos="0 0 0"` to `pos="0 0 1"` to lift the robot above the floor:

```xml
<body name="base_plate" pos="0 0 1" quat="1 0 0 0" childclass="delta-robot">
```

This cannot be set via `config.json` (onshape-to-robot doesn't support body position overrides).

### Self-collision disabled (now in config.json)

The collision mesh geoms originally had default `contype=1, conaffinity=1`, causing every robot part to collide with every other part. Because the mesh geometries overlap in the tightly packed assembly, this generated ~120 spurious contact constraints with forces up to 9 billion, completely locking the mechanism. Now configured in `config.json` via `geom_properties.*.collision.conaffinity="0"` so robot parts only collide with external objects (floor, grasped objects), not with each other.

### Shoulder joint ranges (now in config.json)

The three shoulder hinge joints (`shoulder_1/2/3`) had no `range` attribute, which meant the MuJoCo viewer's Control sliders had zero range. Now configured in `config.json` via `joint_properties.shoulder_*.limits: [-1.5708, 1.5708]` (±90°).

### Actuator control ranges

The shoulder position actuators need `inheritrange="1"` so they inherit the joint range as their control range. This must still be added manually after re-export (onshape-to-robot doesn't support this attribute).

### Actuator gain (now in config.json)

Increased `kp` from 50 to 1000. The equality constraints that close the parallel kinematic loops are very stiff (`solref="0.005 1"`, effective stiffness ~40,000). The original `kp=50` was too weak to drive the coupled mechanism. Now configured in `config.json` via `joint_properties.*.kp: 1000`.

### Simulation options (now in config.json)

Added `<option timestep="0.0005" solver="Newton" iterations="100" tolerance="1e-10"/>`. The Newton solver handles tightly coupled equality constraints better than the default PGS solver, and the smaller timestep prevents numerical instability with the stiff loop-closure constraints (the constraint time constant of 0.005s needs to be well above the timestep). Now configured via `additional_xml: "mujoco_options.xml"`.

**After re-export checklist:**
1. Set `base_plate` body `pos="0 0 1"` (cannot be automated)
2. Add `inheritrange="1"` to shoulder actuators (`shoulder_1`, `shoulder_2`, `shoulder_3`) in the `<actuator>` section (cannot be automated)

## Limitations

- **IK parameter mismatch**: The inverse kinematics parameters (`Fd`, `Ed`, `lower_arm`) were measured from the physical robot and may not perfectly match the simulation geometry. The square trajectory script uses the same parameters as `delta/python/delta_kinematics.py` — if the end effector path looks wrong, these need calibration against the simulation model.
- **No self-collision**: Inter-body collision is disabled for the robot. The robot can still collide with external objects (floor, etc.) but will pass through itself.
- **Soft loop closure**: The closed kinematic chains use MuJoCo equality connect constraints, which are soft (spring-damper). Under large loads or fast motions, small constraint violations (~0.01mm) are normal.
- **Gripper not tuned**: The gripper joints (`flexion_extension`, `pronation_supination`, `gripper`) have actuators but the kinematics/control for them is not implemented in the trajectory scripts.
