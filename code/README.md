# Code Directory

Source code, ROS workspace, and environment tooling.

| Path | Contents |
| ---- | -------- |
| `setup_check.m` | MATLAB environment / toolbox checker. Run once before executing any stage. |
| `ros2_ws/` | ROS 2 workspace for the on-board / HIL bring-up (`src/` — populated during hardware integration). |

The MATLAB simulation stages themselves live in [`../simulation/`](../simulation);
their generated outputs live in [`../data/results/`](../data/results).
