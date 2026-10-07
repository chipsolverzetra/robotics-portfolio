# Robotics Portfolio — Rea Ara

Work samples for robotics expert and AI data annotation roles: runnable
simulations with logged failures, plus a real annotation guideline with
worked examples.

**Rea Ara** — five years as Senior TPM at Google, NYU M.S. Cybersecurity
(in progress). Contact: rea@zetra.dev

## Simulations

### 1. Scripted 2-link arm reaching (MuJoCo) — runs here

`simulations/reacher_mujoco.py` builds a 2-link planar arm in MuJoCo
(MJCF defined inline, no external assets), samples random targets in the
workspace, solves 2-link planar IK analytically, and drives the joints
with PD control.

```
python3 reacher_mujoco.py --trials 30 --out reach_log.csv
```

Latest run: **24/30 successful**, mean final error 1.0 mm on successes.
`simulations/reach_log.csv` is the actual log from that run, with a
failure taxonomy: `unreachable` (target outside the arm's workspace),
`timeout` (no convergence in the step budget).

### 2. Scripted pick-and-place (PyBullet) — work sample

`simulations/pick_place_pybullet.py` runs a KUKA iiwa through a full
pick-and-place: approach, descend, grasp, lift, transport, release,
retreat, with randomized cube starts and per-trial failure logging
(`grasp_miss`, `drop`, `timeout`). Needs `pip install pybullet`
(a local build issue in this sandbox kept it from executing here; the
script is complete and runnable anywhere PyBullet installs).

## Data annotation

- `data-annotation/labeling-guideline.md` — a full labeling guideline
  for robot manipulation episodes (LeRobot / Open X-Embodiment / DROID
  style): episode fields, temporal segment definitions with boundary
  rules, outcome and failure taxonomy, per-frame fields, quality rules.
- `data-annotation/sample_annotations.json` — three episodes annotated
  in that schema: a clean success, a grasp_miss with recovery ending in
  success, and a drop ending in failure.

## Notes

- MuJoCo, PyBullet, and Isaac Sim are all free; these samples use the
  first two. Next: the same reaching task on a simulated SO-100 arm via
  the LeRobot datasets.
- Everything here is original work written for this portfolio.
