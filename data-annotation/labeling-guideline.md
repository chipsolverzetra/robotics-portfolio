# Labeling Guideline: Robot Manipulation Episodes

Version 1.0. Applies to pick-and-place style episodes recorded in the
LeRobot dataset format (MP4 video + parquet with `observation.state`,
`action`, timestamps), and to equivalent episodes from Open X-Embodiment
or DROID after conversion.

## 1. Episode-level fields

| Field | Type | Rule |
|---|---|---|
| `episode_id` | string | Source filename stem, e.g. `so100_pickplace_ep0042`. Never rename. |
| `task_instruction` | string | Copy the instruction verbatim from the episode metadata. Do not paraphrase. |
| `embodiment` | string | Robot + control mode, e.g. `so100_leader_follower`, `kuka_iiwa_scripted`. |
| `episode_outcome` | enum | `success` / `failure` / `ambiguous`. See section 3. |
| `failure_class` | enum or null | Required when outcome is `failure`. See taxonomy below. |

## 2. Temporal segments

Split every episode into contiguous, non-overlapping segments. Boundaries
are the FIRST frame where the new phase's defining condition holds.

| Label | Defining condition |
|---|---|
| `approach` | End effector moving toward the object; gripper open; no contact. |
| `grasp` | Gripper closing OR first contact with the object, until the object moves with the end effector. |
| `transport` | Object rigidly moving with the end effector toward the place target. |
| `place` | End effector descending over the target OR gripper opening to release. |
| `retreat` | Gripper open and empty, moving away from the placed object. |
| `recovery` | Any re-grasp, repositioning, or corrective motion after a failed grasp or drop. |

If a phase never occurs (e.g. grasp fails immediately), omit it. Do not
invent zero-length segments.

## 3. Outcome and failure taxonomy

Mark `success` only if the object ends at rest inside the task's stated
tolerance at the final frame AND no unrecovered drop occurred mid-episode.
A recovered drop that still ends in tolerance is `success` with a
`recovery` segment, not `failure`.

Failure classes:

| Class | Definition |
|---|---|
| `grasp_miss` | Gripper closes with no object contact, or contact without retention. |
| `drop` | Object released or lost before reaching the place target. |
| `collision` | Unintended contact with table, fixture, or self that displaces the scene. |
| `timeout` | Task incomplete when the episode recording ends. |
| `wrong_object` | Grasped or moved an object other than the instructed one. |
| `ambiguous` | Cannot determine outcome from the available views. Flag, do not guess. |

## 4. Per-frame fields (sampled at 10 Hz minimum)

- `gripper_state`: `open` / `closing` / `closed` / `opening`. Judge from the
  gripper, not the action channel: when they disagree, trust the video and
  set `action_video_mismatch: true`.
- `object_in_hand`: boolean. True only when the object moves rigidly with
  the end effector for 3+ consecutive frames.
- `occluded`: boolean. True when the gripper or object is not visible.
  Frames with `occluded: true` must not decide segment boundaries alone;
  extend the boundary to the nearest visible frame and note it.

## 5. Quality rules

1. Never label from the action channel alone; video is ground truth.
2. If the task instruction is missing, set `task_instruction: null` and
   `episode_outcome: ambiguous`. Do not infer the task from the motion.
3. One annotator labels, a second spot-checks 10%. Disagreements on
   segment boundaries within 5 frames are accepted without adjudication.
4. Do not relabel released datasets to "fix" them. Log disagreements in
   `annotator_notes` instead.

## 6. Worked example

`sample_annotations.json` in this folder annotates three episodes in this
schema: one clean success, one grasp_miss with recovery ending in success,
and one drop ending in failure.
