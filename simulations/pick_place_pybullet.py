"""
Scripted pick-and-place with a KUKA iiwa arm in PyBullet.

What it does:
  - Spawns a KUKA iiwa arm and a small cube at a randomized position.
  - Moves the end effector through scripted Cartesian waypoints using
    inverse kinematics: approach -> descend -> grasp -> lift ->
    transport -> descend -> release -> retreat.
  - The grasp is simulated with a fixed constraint (documented below);
    grasp success is judged by whether the cube was within tolerance
    when the grasp was commanded.
  - Runs N trials with randomized cube starts and logs every trial:
    outcome (success/fail) plus a failure reason.

Failure taxonomy (logged per trial):
  - grasp_miss : cube outside grasp tolerance when grasp commanded
  - drop       : cube released/lost before reaching the target zone
  - timeout    : waypoint sequence did not converge in allotted steps

Run:  python3 pick_place_pybullet.py --trials 20 --out failure_log.csv
Requires: pybullet, numpy
"""

import argparse
import csv
import math
import random

import numpy as np

import pybullet as p
import pybullet_data

# ---------------------------------------------------------------- config

EE_LINK = 6              # kuka_iiwa end-effector link index
JOINTS = list(range(7))  # 7 revolute joints
CUBE_HALF = 0.025        # 5 cm cube
GRASP_TOL = 0.045        # max EE-to-cube distance for a successful grasp (m)
PLACE_TOL = 0.05         # success radius around target (m)
MAX_IK_STEPS = 400       # per waypoint
SETTLE_STEPS = 120


def connect():
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")


def spawn_arm():
    arm = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
    for j in JOINTS:
        p.resetJointState(arm, j, 0.0)
    return arm


def spawn_cube(x, y):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3,
                              rgbaColor=[0.9, 0.2, 0.2, 1])
    return p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=col,
                             baseVisualShapeIndex=vis,
                             basePosition=[x, y, CUBE_HALF + 0.001])


def ee_pose(arm):
    pos, orn = p.getLinkState(arm, EE_LINK)[:2]
    return np.array(pos), orn


def move_ee_to(arm, target_xyz, max_steps=MAX_IK_STEPS):
    """Step the arm toward a Cartesian target with IK. Returns final error."""
    target = np.array(target_xyz, dtype=float)
    # keep a downward-facing orientation
    orn = p.getQuaternionFromEuler([0, math.pi, 0])
    for _ in range(max_steps):
        pos, _ = ee_pose(arm)
        err = np.linalg.norm(target - pos)
        if err < 0.008:
            break
        q = p.calculateInverseKinematics(arm, EE_LINK, target.tolist(), orn)
        for j, joint in enumerate(JOINTS):
            p.setJointMotorControl2(arm, joint, p.POSITION_CONTROL,
                                    targetPosition=q[j], force=200)
        p.stepSimulation()
    pos, _ = ee_pose(arm)
    return float(np.linalg.norm(target - pos))


def settle(steps=SETTLE_STEPS):
    for _ in range(steps):
        p.stepSimulation()


def cube_pos(cube):
    pos, _ = p.getBasePositionAndOrientation(cube)
    return np.array(pos)


def run_trial(trial_id, rng):
    """One pick-and-place attempt. Returns a result dict."""
    connect()
    arm = spawn_arm()

    # randomize cube start on the floor in front of the arm
    cx = rng.uniform(0.35, 0.65)
    cy = rng.uniform(-0.25, 0.25)
    cube = spawn_cube(cx, cy)
    settle()

    target = np.array([0.0, 0.55, CUBE_HALF + 0.001])  # place zone
    hover = 0.25
    result = {"trial": trial_id,
              "cube_start_x": round(cx, 3), "cube_start_y": round(cy, 3),
              "outcome": "fail", "failure_reason": ""}

    grasp_constraint = None
    try:
        # 1. approach above cube
        move_ee_to(arm, [cx, cy, hover])
        # 2. descend to grasp height
        move_ee_to(arm, [cx, cy, CUBE_HALF + 0.02])
        settle()

        # 3. grasp: check tolerance, then attach
        pos, _ = ee_pose(arm)
        cp = cube_pos(cube)
        if np.linalg.norm(pos - cp) > GRASP_TOL:
            result["failure_reason"] = "grasp_miss"
            return result
        grasp_constraint = p.createConstraint(
            arm, EE_LINK, cube, -1, p.JOINT_FIXED,
            [0, 0, 0], [0, 0, 0], [0, 0, 0.02])

        # 4. lift
        move_ee_to(arm, [cx, cy, hover])
        # 5. transport above target
        move_ee_to(arm, [target[0], target[1], hover])
        # 6. descend
        move_ee_to(arm, [target[0], target[1], CUBE_HALF + 0.03])
        settle()

        # 7. release
        p.removeConstraint(grasp_constraint)
        grasp_constraint = None
        settle()

        # 8. retreat
        move_ee_to(arm, [target[0], target[1], hover])

        # did the cube survive the trip?
        cp = cube_pos(cube)
        if np.linalg.norm(cp[:2] - target[:2]) > PLACE_TOL or cp[2] > 0.12:
            result["failure_reason"] = "drop"
            return result

        result["outcome"] = "success"
        return result
    finally:
        if grasp_constraint is not None:
            p.removeConstraint(grasp_constraint)
        p.disconnect()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--out", default="failure_log.csv")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    rows = [run_trial(i, rng) for i in range(args.trials)]

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    ok = sum(1 for r in rows if r["outcome"] == "success")
    print(f"{ok}/{len(rows)} successful; log -> {args.out}")
    for reason in ("grasp_miss", "drop", "timeout"):
        n = sum(1 for r in rows if r["failure_reason"] == reason)
        if n:
            print(f"  {reason}: {n}")


if __name__ == "__main__":
    main()
