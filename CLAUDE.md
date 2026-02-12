# Delta Policy

This repo trains a learned visuomotor policy for a delta robot equipped with a Koch-arm style gripper end effector.

## Hardware Setup

- **Robot**: Delta robot with a Koch-arm style gripper
- **End-effector camera**: Mounted on the gripper for close-up / in-hand views
- **Overhead camera**: Positioned above the workstation (under the delta robot base) for a top-down workspace view

## Open Design Decisions

- **Teleoperation / demonstration method**: Not yet decided. Candidates include ALOHA-style demonstrations, VR controller, or spacemouse with keyboard/pedal.
- **Policy architecture**: Not yet decided. Candidates include Diffusion Policy, ACT, VLAs, or other behavior cloning approaches.
