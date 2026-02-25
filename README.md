
<img width="886" height="500" alt="Screenshot 2026-02-25 at 08 33 52" src="https://github.com/user-attachments/assets/5abbc4db-d20a-4807-98a7-16c55268eac7" />

# Delta Robot Simulation

MuJoCo simulation of a self-built delta robot. The MJCF model (`simulation/delta-robot/robot.xml`) was auto-generated from the Onshape CAD assembly using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot). Note that the mates need to follow certain naming conventions for this to work, check their docs. 

[Onshape source](https://cad.onshape.com/documents/20bfbde5f0dfed09746f3835/w/89e87a07589996b404d06798/e/02e6df3ca634eef0ea4fbf46)

Also, for fun I added the [Koch arm](https://github.com/AlexanderKoch-Koch/low_cost_robot) to the end effector.

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

- **IK parameter mismatch**: The inverse kinematics parameters (`Fd`, `Ed`, `lower_arm`) were measured from the physical robot and don't perfectly match the simulation geometry. I've just been a little bit lazy. Might update the values to be exact.
- **No self-collision**: Inter-body collision is disabled for the robot. The robot can still collide with external objects (floor, etc.) but will pass through itself (so it will not have the same workspace constraints as a physical version).
- **Soft loop closure**: The closed kinematic chains use MuJoCo equality connect constraints, which are soft (spring-damper). Under large loads or fast motions, small constraint violations (~0.01mm) are normal.
- **Gripper not tuned**: The gripper joints (`flexion_extension`, `pronation_supination`, `gripper`) have actuators but the kinematics/control for them is not implemented in the trajectory scripts.

## Real version
I made a real version of this robot too. Reach out if you have any questions regarding that. 
![65F75180-BE6D-424E-82A5-F1F4A6366AF7_1_105_c](https://github.com/user-attachments/assets/3e949cb9-f0ee-498a-bb72-7e3726d2f7b0)


