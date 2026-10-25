---
name: feedback-minimal-setup
description: Keep the project minimal and goal-driven, and prefer open-source code we can read over Unitree's closed services
metadata:
  type: feedback
---

Build only what the current concrete goal needs. Don't add architecture ahead of decisions: no multi-node skeletons, custom messages or deploy tooling until a goal requires them. Prefer open-source code we can read, and keep it in the project: NVIDIA's GR00T policy is our walking controller ([[gr00t-wbc]]). Avoid Unitree's closed services (sport mode, LocoClient and the like) where possible. Unitree's open pieces are fine to use: the message definitions, unitree_sdk2_python, and the MuJoCo model. Prefer official sources over random community libraries.

**Why:** On 2026-10-24 a teammate pushed back on a full ROS 2 skeleton an agent had built ("you've put a bunch of code") and narrowed the scope to one goal: walk a set distance. On 2026-10-25 the mentors asked the team to move off Unitree's closed libraries and onto open code such as GR00T.

**How to apply:** Start from an official example. Add the smallest amount of our own code, and explain what each piece is for. When a goal is reached, ask before expanding the structure. ROS 2 was the original requirement, but our current scripts use unitree_sdk2_python directly. Whether ROS 2 stays required is undecided, so ask before adding or dropping it. See [[project-overview]].
