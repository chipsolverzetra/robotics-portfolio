"""
Scripted 2-link arm reaching in MuJoCo.

What it does:
  - Builds a 2-link planar arm (MJCF defined inline, no external assets).
  - Each trial samples a random target in the reachable workspace.
  - Solves 2-link planar inverse kinematics analytically, then drives
    the joints to the IK solution with PD control.
  - Logs every trial: target, outcome (success/fail), failure reason,
    final end-effector error.

Failure taxonomy:
  - unreachable : sampled target outside the arm's reachable annulus
                  (resampled, logged for transparency)
  - timeout     : joints did not converge within the step budget

Run:  python3 reacher_mujoco.py --trials 30 --out reach_log.csv
Requires: mujoco, numpy
"""

import argparse
import csv
import math
import random

import numpy as np

import mujoco

MJCF = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <light name="top" pos="0 0 2"/>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="shoulder_base" pos="0 0 0.5">
      <joint name="shoulder" type="hinge" axis="0 1 0"
             limited="true" range="-170 170" damping="1.0" armature="0.05"/>
      <geom name="link1" type="capsule" fromto="0 0 0 0.30 0 0" size="0.02"/>
      <body name="elbow_body" pos="0.30 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"
               limited="true" range="-170 170" damping="0.8" armature="0.03"/>
        <geom name="link2" type="capsule" fromto="0 0 0 0.25 0 0" size="0.015"/>
        <body name="ee" pos="0.25 0 0">
          <geom name="ee_dot" type="sphere" size="0.015" rgba="0 0 1 1"/>
          <site name="ee_site" pos="0 0 0"/>
        </body>
      </body>
    </body>
    <body name="target" mocap="true" pos="0.4 0 0.3">
      <geom name="target_dot" type="sphere" size="0.02" rgba="1 0 0 0.8"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_shoulder" joint="shoulder" gear="20"/>
    <motor name="m_elbow" joint="elbow" gear="12"/>
  </actuator>
</mujoco>
"""

L1, L2 = 0.30, 0.25
SHOULDER_Z = 0.5
SUCCESS_TOL = 0.02
STEPS = 1500
KP, KD = 25.0, 2.5


def ik_2link(x, z):
    """Analytic IK for planar 2-link arm. Returns (shoulder, elbow) or None."""
    r = math.hypot(x, z)
    if r > L1 + L2 or r < abs(L1 - L2) + 1e-6:
        return None
    d = max(-1.0, min(1.0, (x * x + z * z - L1 * L1 - L2 * L2)
                       / (2 * L1 * L2)))
    elbow = math.atan2(-math.sqrt(1 - d * d), d)  # elbow-down
    shoulder = (math.atan2(z, x)
                - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow)))
    return shoulder, elbow


def run_trial(trial_id, rng, force_unreachable=False):
    model = mujoco.MjModel.from_xml_string(MJCF)
    data = mujoco.MjData(model)

    # sample target relative to shoulder joint
    tx = rng.uniform(0.15, 0.50)
    tz = SHOULDER_Z + rng.uniform(-0.35, 0.05)
    if force_unreachable:
        # deliberately outside the reachable annulus: documents the
        # "unreachable" failure class in the log
        tx = rng.uniform(0.70, 0.90)
        tz = SHOULDER_Z + rng.uniform(0.10, 0.30)
    dx, dz = tx - 0.0, tz - SHOULDER_Z

    result = {"trial": trial_id,
              "target_x": round(tx, 3), "target_z": round(tz, 3),
              "outcome": "fail", "failure_reason": "",
              "final_error_m": ""}

    sol = ik_2link(dx, dz)
    if sol is None:
        result["failure_reason"] = "unreachable"
        return result
    # Joint rotates +X toward -Z for positive q, opposite the IK frame,
    # so negate the analytic solution.
    q_sh, q_el = (-sol[0], -sol[1])

    data.mocap_pos[0] = [tx, 0.0, tz]

    for _ in range(STEPS):
        # PD control toward IK solution
        for i, q_target in enumerate((q_sh, q_el)):
            q = data.qpos[i]
            qd = data.qvel[i]
            data.ctrl[i] = KP * (q_target - q) - KD * qd
        mujoco.mj_step(model, data)

    ee = data.site("ee_site").xpos
    err = float(np.linalg.norm(np.array([tx, 0.0, tz]) - ee))
    result["final_error_m"] = round(err, 4)
    if err <= SUCCESS_TOL:
        result["outcome"] = "success"
    else:
        result["failure_reason"] = "timeout"
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--out", default="reach_log.csv")
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    rows = [run_trial(i, rng, force_unreachable=(i % 5 == 4))
            for i in range(args.trials)]

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    ok = sum(1 for r in rows if r["outcome"] == "success")
    errs = [r["final_error_m"] for r in rows if r["final_error_m"] != ""]
    print(f"{ok}/{len(rows)} successful; log -> {args.out}")
    if errs:
        print(f"  mean final error: {np.mean(errs):.4f} m")
    for reason in ("unreachable", "timeout"):
        n = sum(1 for r in rows if r["failure_reason"] == reason)
        if n:
            print(f"  {reason}: {n}")


if __name__ == "__main__":
    main()
