# Swarm Mode Design — ArduPilot

Summary
-------
Goal: add a new flight mode / library so vehicles can operate in a small swarm. Instead of following a single leader, each vehicle ingests position/velocity/attitude data from multiple vehicles (distinguished by MAVLink sysid) and computes a desired setpoint that:
- drives the swarm towards a GCS-specified location (the leader / goal),
- enforces pairwise separation constraints (so vehicles keep a given distance from each other),
- is robust to packet loss, varying update rates, missing GPS, and different alt frames.

Recommended approach
--------------------
1. New library: AP_Swarm
   - Keeps the swarm code separated from AP_Follow.
   - Handles multi-sysid message tracking, per-sysid estimators, formation logic, setpoint generation.
2. Use a hybrid formation control:
   - The GCS sets an explicit goal location (the "target position" for the swarm). The vehicle with sysid 1 (the leader) is expected to go to that coordinate.
   - Every other vehicle computes a desired setpoint based on:
     - attractive term to the leader/goal,
     - deterministic formation assignment (e.g., ring/circle offsets computed from sysid or index),
     - local neighbor-spacing (repulsive) term for robustness,
     - alt-frame handling (configurable).
   - Deterministic assignment reduces ambiguity (no need for expensive consensus). For small swarms, a ring/circle or radial/slot assignment is simple and robust.

Why not just "follow everyone and maintain distance from all"?
- Doing pairwise constraints alone (only repulsive terms) can be underspecified: without a centralized or deterministic assignment, decentralized agents can oscillate or produce collisions in symmetric situations.
- Deterministic assignment (based on sysid → slot) combines simplicity and stability; repulsive neighbor terms help handle disturbances and minor deviations.

High-level architecture
-----------------------
- AP_Swarm (new library)
  - Per-sysid TargetEntry struct:
    - sysid (uint8_t)
    - last_pos, last_vel, last_accel (NEU / NED consistent frame)
    - last_heading, last_heading_rate
    - last_update_ms
    - jitter correction and a copy of the per-target estimator state (pos/vel/accel estimates)
    - valid flags, timeouts
  - Container: vector or fixed-size array of TargetEntry (bounded by a configurable MAX_NEIGHBORS).
  - GCS inputs:
    - Goal location (global lat/lon/alt) — could be set via MAVLink from GCS or read from leader sysid==1 position if you want.
    - Formation parameters (formation_type, radius, spacing, ordering).
  - Parameters:
    - SWARM_ENABLE
    - SWARM_FORMATION_TYPE (circle/line/grid/custom)
    - SWARM_RADIUS (for circle)
    - SWARM_SLOT_OFFSET (per-vehicle offset option)
    - SWARM_NEIGHBOR_TIMEOUT_MS
    - SWARM_MAX_NEIGHBORS
    - SWARM_ALT_FRAME
    - Gains: attract_gain, repel_gain, damping
    - Kinematic shaping params (or reuse AP_Follow params)
  - Functions:
    - message handler: accept MAVLink GLOBAL_POSITION_INT / FOLLOW_TARGET from any sysid (subject to filtering per-params)
    - per-loop update: update per-target estimators, drop stale targets
    - compute desired setpoint for this vehicle
    - output location/velocity setpoint for autopilot position controller or set waypoint in local controller (depends on integration choice)

Message handling & inputs
-------------------------
- Accept messages from multiple sysids. Do not filter to only one sysid. Keep per-sysid state updated by last-seen timestamps.
- Support GLOBAL_POSITION_INT and FOLLOW_TARGET (FOLLOW_TARGET has more fields: attitude, acc, vel). If FOLLOW_TARGET received, use its richer data. GLOBAL_POSITION_INT is fallback.
- Time sync: reuse AP_Follow jitter-corrector approach for each sysid. Each TargetEntry should run its own jitter corrector.
- Store messages in local TargetEntry; compute per-target kinematic projection/estimation using same shaping/jerk-limited approach as AP_Follow.
- Implement neighbor timeout: if last_update older than neighbor_timeout → drop or mark invalid.

Per-target estimation
---------------------
- Reuse the kinematic shaping/estimation algorithm used in AP_Follow. It provides:
  - projection of pos/vel/accel using last-known velocity/acceleration,
  - shaping toward new data using accel/jerk limits,
  - heading estimation.
- To make code maintainable: extract AP_Follow estimation code into a reusable "KinematicEstimator" or "TargetEstimator" class that can be instantiated per sysid. This will allow both AP_Follow (single-target) and AP_Swarm (multiple targets) to share logic.
- If significant refactor is not desired up front, copy the minimal estimation functions into AP_Swarm initially — but refactor later.

Formation and setpoint generation ideas
---------------------------------------
Two practical options:

A) Centralized (GCS assigns absolute desired positions to each vehicle)
   - GCS calculates exact coordinates for each vehicle and transmits them (e.g. via MAVLink messages or a parameter upload).
   - Pros: simplest to control precise formation, no complex on-flight assignment.
   - Cons: requires GCS to compute/communicate positions; single point of failure; more message bandwidth.

B) Deterministic decentralized assignment + leader goal (recommended for initial implementation)
   - GCS broadcasts a single goal position or instructs sysid 1 to go to the goal.
   - Each vehicle deterministically maps its sysid (or position in the known list) to a formation slot. Examples:
     - Circle formation: angle = 2π * (index / N), desired offset = radius * [cos(angle), sin(angle)] relative to leader.
     - Radial formation: computed radial slot by index.
   - The formation is centered on the leader (or a formation centroid computed from leader + params).
   - This avoids complicated consensus and is robust for small/medium swarms.
   - The algorithm must be deterministic and consistent across vehicles (they must compute the same ordering). Ordering alternatives:
     - Ordering by sysid ascending after ignoring self (the same deterministic sort on every vehicle).
     - Use only vehicles that are "active" (have valid target entries) and sort their sysids.

Setpoint computation (control law)
----------------------------------
Compute desired position x_des as:

x_des = X_goal + slot_offset   (slot_offset depends on formation assignment)

To add robustness, blend in repulsive neighbor terms to keep pairwise distances:

x_des = X_goal + slot_offset + K_rep * sum_j ( unit_vector( x - x_j ) * (d_ij - |x - x_j| )_+ )

Where:
- X_goal = leader position / global goal
- slot_offset = deterministic offset computed from sysid (in leader frame: rotate by leader heading if needed)
- x_j = estimated neighbor j position (NED)
- d_ij = desired distance between i and j (maybe same constant or derived from formation)
- (a)_+ = max(0, a) ensures repulsion only when closer than desired
- K_rep is a tuning gain
- Also apply damping to avoid oscillations (velocity damping using vel terms)

A simpler version:
- Use x_des = leader_pos + slot_offset. Use the repulsive neighbor term only as a collision-avoidance fallback (active only if neighbor is too close).
- If you want continuous formation maintenance without explicit slot offsets, use attractive-to-goal + pairwise spring/repulsive forces (but that can be unstable/slow for symmetric configurations).

Altitude handling
-----------------
- Follow AP_Follow patterns: allow configuration of alt frame (absolute vs above-home).
- Allow the formation to have a separate formation altitude parameter or use leader's altitude plus an offset.

Integration with vehicle setpoints
----------------------------------
- AP_Swarm could output:
  - a global/local Location setpoint (best) to the autopilot position controller,
  - or set an auto-mode style target (e.g., update the guided target),
  - or publish a MAVLink TARGET_* message if desired.
- Keep the integration path consistent with ArduPilot's existing position control (e.g., set guided position) to let built-in controllers handle lower-level control.

Parameters to add
-----------------
- SWARM_ENABLE (bool)
- SWARM_FORMATION_TYPE (enum: Disabled/LeaderOnly/CIRCLE/LINE/GRID/...)
- SWARM_RADIUS (for circle) or SWARM_SPACING (for grid/line)
- SWARM_LEADER_SYSID (default 1)
- SWARM_INDEX_BY (enum: sysid / sorted_sysid / dynamic)
- SWARM_NEIGHBOR_TIMEOUT_MS (default 10000)
- SWARM_MAX_NEIGHBORS (for memory bounds)
- SWARM_ATTRACT_GAIN
- SWARM_REPEL_GAIN
- SWARM_REPEL_DISTANCE (min allowed distance)
- SWARM_ALT_TYPE (copy from AP_Follow)
- optionally: SWARM_LOG_LEVEL, SWARM_DEBUG

Safety and edge cases
---------------------
- Loss of GPS: if a vehicle has no reliable position, it should drop out or hover safely. The swarm code must not command unsafe behavior when neighbor positions are stale.
- Packet loss / varying update rates: use per-target timeouts; remove stale targets. Consider smoothing (kinematic estimator) to reduce jitter.
- Bandwidth: multiple vehicles broadcasting high-rate POSITION messages can overwhelm low-bandwidth links. Limit accepted sysids or throttle processing; use compact messages or reduce message rate. Consider optional group-based broadcast vs. all-to-all.
- Conflicts & collisions: repulsive term helps but is not a full anti-collision system. Consider enabling a strict proximity abort (if any neighbor < safety_min_distance -> hover or land).
- Formation consistency: deterministic assignment requires same active list everywhere. Ensure sorting and active filtering rules are identical on each vehicle.
- Leader changes: support a new leader if the GCS or operator designates one.
- Geo-fence / failsafe: if the swarm setpoint conflicts with geo-fence or legal restrictions, the vehicle's built-in failsafes should still apply.
- Simulation testing (SITL) is essential before hardware testing.

Scalability & performance
-------------------------
- Keep neighbor list bounded (e.g., 20). Use fixed-size arrays to avoid dynamic allocation on embedded platform.
- Do light-weight math: estimator tick rate can be lower than main loop; design for efficient vector ops.
- Consider distributing formation info via MAVLink parameter sync or a custom MAVLink msg if needed.

Logging & debugging
-------------------
- Add logging similar to AP_Follow (FOLL) to capture per-vehicle estimate, neighbor list, computed setpoint, and formation assignments.
- Provide GCS telemetry messages for operator awareness (current formation, active neighbors).

Suggested incremental implementation plan
-----------------------------------------
1. Prototype as a separate library AP_Swarm with:
   - per-sysid TargetEntry
   - message handler to capture GLOBAL_POSITION_INT and FOLLOW_TARGET into entries
   - simple "leader-only" behavior: compute setpoint as leader_pos + slot_offset (circle formation) and command it to controller (local position setpoint). No per-neighbor repulsion yet.
   - add parameters: SWARM_ENABLE, SWARM_FORMATION_TYPE, SWARM_RADIUS, SWARM_LEADER_SYSID, SWARM_NEIGHBOR_TIMEOUT_MS.
   - test in SITL with a few simulated vehicles broadcasting GLOBAL_POSITION_INT.
2. Add per-target kinematic estimator:
   - either refactor AP_Follow code into a TargetEstimator class or copy required logic initially
   - ensure jitter-correction per-target.
3. Add neighbor sorting and deterministic assignment:
   - collect active sysids, sort, compute index.
4. Add repulsive neighbor term for collision robustness and tuning parameters.
5. Add safety checks/timeouts, logging, and tuned param defaults.
6. Add unit tests & SITL scenarios. Test edge cases: missing leader, packet loss, leader moves abruptly.
7. (Optional) Refactor AP_Follow to share the estimator class.

Refactor recommendation (for long-term)
--------------------------------------
- Extract per-target kinematic estimation (projection + shaping + heading logic) from AP_Follow into a small class (e.g., TargetEstimator). AP_Follow becomes a thin wrapper that owns a single TargetEstimator; AP_Swarm owns multiple TargetEstimator instances.
- That reduces duplication and keeps behavior consistent across Follow & Swarm.

MAVLink considerations
---------------------
- Use FOLLOW_TARGET where available since it supplies richer state. Fallback to GLOBAL_POSITION_INT.
- Message rates: keep per-vehicle publish rates moderate (1–5 Hz) depending on formation dynamics and communication capacity.
- If GCS needs to push formation or goal, consider:
  - A custom MAVLink message to broadcast swarm formation config, or
  - Use existing parameter setting with rapid param push to the swarm group, or
  - Use COMMAND_LONG or STATUSTEXT (less ideal).
- Consider adding a new MAVLink message or extending companion communication for large swarms / advanced configs.

Example pseudo-code (per-loop)
------------------------------
This is an outline of the update loop you can implement in AP_Swarm:

1. On MAVLink msg (GLOBAL_POSITION_INT/FOLLOW_TARGET):
   - find / create TargetEntry for msg.sysid
   - decode fields into entry raw values
   - apply jitter correction to timestamp
   - mark last_update_ms

2. Periodic update (e.g., 10 Hz):
   - current_ms = now
   - remove stale targets (now - last_update_ms > neighbor_timeout)
   - for each active entry:
       entry.estimator.update_with_new_observation(...)
       entry.estimate_project_forward(dt)
   - compute active_sysid_list sorted ascending
   - find my index in active list -> slot_index
   - compute slot_offset = formation_slot(slot_index, formation_type, radius)
   - leader_pos = get leader position from active list (use SWARM_LEADER_SYSID)
   - desired_pos = leader_pos + rotate_by_leader_heading(slot_offset)
   - for each neighbor j:
       if distance to j < repel_threshold:
          desired_pos += repel_gain * unit_vector(my_pos - pos_j) * (repel_threshold - dist)
   - output desired_pos to autopilot as guided/local setpoint
