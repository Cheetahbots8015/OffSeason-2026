# Vision & Odometry Investigation — OffSeason 2026

Date: 2026-07-21
Goal: fix (1) odometry drift after collisions and (2) poor pose quality when the
Limelights cannot see a reliable target, by adopting proven practices from top FRC teams.

Sources audited:
- This repo: `Drive.java`, `Robot.java`, `DriveConstants.java`, `GyroIOPigeon2.java`
- Limelight official MegaTag2 docs — https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
- Limelight MegaTag(1) docs — https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
- LimelightHelpers javadoc — https://limelightlib-wpijava-reference.limelightvision.io/frc/robot/LimelightHelpers.html
- FRC 6328 (Mechanical Advantage) 2026 public code "Darwin" — https://github.com/Mechanical-Advantage/RobotCode2026Public
  - `subsystems/vision/Vision.java`, `subsystems/vision/VisionConstants.java`, `RobotState.java`
- FRC 6328 2025 build thread (pose estimation design) — https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314
- PhotonVision pose estimator docs — https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
- WPILib pose estimator docs — https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html

---

## 1. Current implementation audit

Vision polling lives inline in `Drive.periodic()` (`Drive.java:320-416`), as three
near-identical copy-pasted blocks for `limelight-swerve`, `limelight-left`,
`limelight-rear`, fused into a `SwerveDrivePoseEstimator` with theta stddev ~1e7
(vision yaw not trusted — correct).

### Bugs and weaknesses found

**A. Camera name mismatch — IMU config is applied to cameras that are never polled.**
`Robot.java:150-151` calls `SetIMUMode("limelight-shooter", 1)` and
`SetIMUMode("limelight-chassis", 1)`, but `Drive.java` polls `limelight-swerve`,
`limelight-left`, `limelight-rear`. Either the web-UI hostnames were renamed and the
init calls are stale no-ops, or the polled cameras do not exist and every poll throws
(swallowed by the empty catch blocks). Net effect: the cameras actually in use run the
default IMU mode (0, EXTERNAL_ONLY), and the intended seeding never happens.

**B. No gyro-rate gating — Limelight's #1 recommended MT2 filter is missing.**
The official MT2 example rejects vision updates when `Math.abs(gyro.getRate()) > 360`
deg/s (some team versions use 720 deg/s). During and right after a collision the robot
is spun around; MT2's yaw seed lags one frame, so exactly those frames produce bad
poses — and we currently accept all of them.

**C. The stddev model cannot correct the pose after a collision.**
`calculateStdDevs` (`Drive.java:240-260`) computes `avg(dist * ambiguity)` and clamps
it to **[0.5, 5]**. A 0.5 m floor means that even a perfect 3-tag fix at 1 m is trusted
as if it were ±0.5 m. 6328's model (`Vision.java`) is:

```
xyStdDev    = 0.01 * avgTagDist^2 / tagCount^2 * cameraStdDevFactor
thetaStdDev = (multi-tag) ? 0.03 * avgTagDist^2 / tagCount^2 : +infinity
```

i.e. ~0.001–0.01 m at close range — roughly **100–400× more trust** in good vision
than our floor. This is the single biggest reason our pose stays wrong after a hit:
good corrections arrive but are blended away at ~4% Kalman gain per frame.

**D. Dead sentinel logic.**
`calculateStdDevs` returns `999999` when StdDev > 5 (`Drive.java:256-258`), but all
three call sites check `if (StdDev != 9999999)` (six vs seven nines — `Drive.java:340,
373, 406`). The reject path never triggers. (Mostly harmless — a 1e6 stddev is a no-op
correction anyway — but it is dead, misleading code.)

**E. MT2 orientation seeding uses the smoothed estimate and no yaw rate.**
`SetRobotOrientation` is fed `poseEstimator.getEstimatedPosition().getRotation()` and
`yawRate = 0` (`Drive.java:321-328` etc.). The estimator rotation lags the gyro and is
wrong while the gyro-disconnected fallback is active. `GyroIO` already provides
`yawVelocityRadPerSec` (`GyroIOPigeon2.java:54`) — seed with **raw gyro yaw + yaw
rate** instead; MT2 interpolates yaw at frame capture time and measurably improves
while rotating.

**F. Silent failure everywhere.**
All three camera blocks end in `catch (Exception e) {}`. A disconnected or misnamed
camera, a null `PoseEstimate`, or an NPE on `mt2.pose` logging is invisible. There is
a disconnected-gyro alert but no per-camera alert, and rejection reasons are not
logged — which makes field debugging ("is the camera blind or is the code rejecting?")
guesswork.

**G. Missing standard sanity filters.** No field-bounds rejection (6328 uses a 0.5 m
margin outside the field rectangle), no z-range check, and `DriveConstants.maxAmbiguity
/ maxCameraDist / minArea` exist but are unused — `shouldReject` hardcodes 0.5 / 4.0.

**H. Structure.** Vision is triplicated inside the drive subsystem instead of a
dedicated `Vision` subsystem with per-camera config — the pattern every reference
codebase (6328, and the AdvantageKit template this repo is built on) uses.

---

## 2. What strong teams do

### Limelight official guidance (MT2 docs)
- Reject updates when |gyro rate| > 360 deg/s; reject when tagCount == 0.
- Baseline stddevs `.7, .7, 9999999` are explicitly a *starting point* to be tuned down.
- IMU modes (LL4): 0 EXTERNAL_ONLY, 1 EXTERNAL_SEED (use while **disabled**, keeps
  internal IMU seeded), 2 INTERNAL_ONLY, 3 INTERNAL_MT1_ASSIST, 4
  INTERNAL_EXTERNAL_ASSIST (recommended while **enabled**: 1 kHz internal IMU with
  gentle drift correction from the robot gyro, alpha default 0.001).
- Restrict tag IDs dynamically (`SetFiducialIDFiltersOverride`), throttle cameras while
  disabled to reduce heat, upload an accurate `.fmap`, and configure each camera's
  robot-space pose in the web UI.

### 6328 Mechanical Advantage (2026 "Darwin")
- Dedicated `Vision` subsystem; per-camera `CameraConfig` (pose, exposure, gain, FOV,
  per-camera `stdDevFactor`), all observations sorted by timestamp before fusion.
- Filters: ambiguity disambiguation (threshold 0.4, pick the solution closest to the
  current gyro rotation), field-border margin 0.5 m, z in [-0.5, 1.0] m, vision ignored
  during the first 2 s of auto.
- Stddev model: `0.01 * avgDist² / n²` (xy), `0.03 * avgDist² / n²` (theta, multi-tag
  only; single-tag theta = ∞). Per-camera scale factor.
- Custom fusion in `RobotState`: odometry twists advance both `odometryPose` and
  `estimatedPose`; a 2 s `TimeInterpolatableBuffer` stores odometry history; each
  vision observation rewinds the estimate to the observation timestamp, applies a
  closed-form scalar Kalman gain `K = q/(q + sqrt(q·r))` per axis, then replays
  odometry forward. Odometry state stddevs are tiny: (0.003, 0.003, 0.002).
- **Tilt scaling:** when roll/pitch indicates the robot is not flat (ramps, climbing),
  the translation component of the odometry twist is scaled toward 0 (wheels spinning
  in the air do not move the pose). Same family of fix as "wheel-slip" handling.
- Per-camera disconnect timers + alerts; rich per-frame logging.

### PhotonVision ecosystem (for reference)
- MultiTag is the recommended strategy; **Constrained SolvePnP** solves single-tag pose
  on the RIO using the gyro heading as a constraint — the same idea as MT2 — and is the
  recommended fallback when not using Limelights.

### On collisions specifically
No top team "detects the collision and resets." The published pattern is:
1. Make good vision strong enough that the pose re-converges within a few frames
   (6328's stddev model above).
2. Reject vision while spinning fast (gyro-rate gate), since post-impact rotation is
   when MT2 data is worst.
3. De-weight odometry when it is physically unreliable: tilt scaling (6328), or
   per-module slip outlier rejection — the "skid detection" idea credited to FRC 1690
   Orbit, where a module whose measured delta disagrees with the twist solved from the
   other modules is dropped for that sample.

---

## 3. Recommended changes (priority order)

### P0 — correctness (do first, small diffs)
1. **Fix the camera name mismatch** (`Robot.java:150-151` vs `Drive.java`). One source
   of truth for camera names in `DriveConstants`/a new `VisionConstants`.
2. **Gyro-rate gate:** reject vision when
   `Math.abs(gyroInputs.yawVelocityRadPerSec) > Units.degreesToRadians(360)` (tunable
   up to 720). This directly targets post-collision corruption.
3. **Replace the stddev model** with distance-squared scaling (MT2-tuned starting
   point — note 6328's 0.01 coefficient is for their own solver, MT2 needs a bit more):
   ```java
   double xyStdDev = Math.max(0.03, 0.08 * Math.pow(avgTagDist, 2)
       / Math.pow(tagCount, 2) * cameraStdDevFactor);
   poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
       VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
   ```
   Use the per-measurement `addVisionMeasurement(pose, timestamp, stdDevs)` overload
   and delete `setVisionMeasurementStdDevs` state-sharing between cameras.
4. **Fix or delete the `999999` vs `9999999` sentinel** (currently dead logic).
5. **Seed MT2 with raw gyro yaw + yaw rate** instead of the estimator rotation:
   `SetRobotOrientation(name, gyroInputs.yawPosition.getDegrees(),
   Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec), 0, 0, 0, 0)`.
6. **IMU mode lifecycle (LL4 only — verify hardware):** mode 1 while disabled
   (`disabledPeriodic`), switch to mode 4 when enabled, on the *actual* camera names.
7. **Stop swallowing exceptions**; log them plus a per-camera "rejected reason" enum.

### P1 — robustness & observability
8. **Extract a `Vision` subsystem** (`Vision`, `VisionIO`, `VisionIOLimelight`,
   per-camera constants incl. stdDevFactor) — removes the triplicated blocks and
   matches the AdvantageKit pattern already used everywhere else in this repo.
9. **Per-camera disconnect alerts** (no frames for >1.5 s) and log per-camera:
   accepted pose, stddev used, rejection reason, tag count, avg dist, latency.
10. **Field-bounds rejection:** drop poses more than 0.5 m outside the field rectangle.
11. Consolidate acceptance constants into one place (reuse `DriveConstants.maxAmbiguity`
    etc. or move to `VisionConstants`); scale max tag distance by tag count
    (single tag ~3.5–4 m; multi-tag can be accepted farther).

### P2 — collision/slip hardening
12. **Impact-aware vision boost (simple, effective):** detect impacts via Pigeon2
    accelerometer jerk or a yaw-rate spike inconsistent with commanded omega; for
    ~0.5 s after impact multiply vision stddevs by ~0.5 (trust vision more) so the pose
    snaps back fast. Log every impact event.
13. **Module-slip outlier rejection (1690-style):** per odometry sample, solve the
    twist, compare each module's measured delta to the twist-predicted delta; if one
    module is a large outlier, re-solve without it (or halve its weight). This stops a
    skidding wheel from dragging the pose during a shove.
14. **Tilt scaling (6328-style)** if the 2026 game has ramps: scale twist translation
    down as roll/pitch depart from flat.
15. **Camera-side tuning checklist:** lowest exposure that still detects tags
    reliably (6328 runs ~1800 µs mono, gain ~15), AprilTag 36h11 pipeline, sensible
    downscaling for range vs fps, rigid mounts, correct robot-space pose per camera,
    current `.fmap`, throttle while disabled.

### Verification plan
- Log everything first (P1 #9), then in AdvantageScope: plot `LL/*/pose` vs
  `Odometry/Robot`; (a) spin in place — vision updates must stop above the rate gate
  and resume cleanly; (b) shove the robot sideways with wheels spinning — pose must
  re-converge in < ~0.5 s once a tag is visible; (c) drive out of tag range — pose
  must coast on odometry without jumps.
- Tune the stddev baseline by watching convergence speed vs jitter trade-off on a
  known field position.

---

## 4. Expected impact on the two reported symptoms

| Symptom | Root cause found | Fix that addresses it |
|---|---|---|
| Odometry off after collisions | Vision trust floor of 0.5 m (C) + no gyro-rate gate (B) + no slip handling | P0-2, P0-3, P0-5, P2-12/13 |
| Pose bad when no reliable target | Silent camera failures (A, F), no disconnect alerts, over-aggressive rejection constants hardcoded (G), weak seeding (E) | P0-1, P0-7, P1-9/10/11 |
