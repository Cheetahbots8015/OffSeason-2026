package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class VisionConstants {
  /**
   * Per-camera config. stdDevFactor scales the vision stddev model per camera (raise for a camera
   * with a worse mount/calibration). isLL4 gates IMU mode 4, which only exists on Limelight 4.
   */
  public record CameraConfig(String name, boolean isLL4, double stdDevFactor) {}

  public static final CameraConfig[] cameras = {
    new CameraConfig("limelight-swerve", false, 1.0),
    new CameraConfig("limelight-left", false, 1.0),
    new CameraConfig("limelight-rear", true, 1.0),
  };

  // Reject all vision frames while rotating faster than this. MT2's yaw seed lags one
  // frame, so frames captured during fast rotation produce bad poses — exactly the
  // post-collision window. Limelight's docs suggest 360 deg/s; some teams run 720.
  public static final double maxGyroRateRadPerSec = Units.degreesToRadians(360.0);

  // Acceptance filters
  public static final double maxAmbiguity = 0.5;
  public static final double maxSingleTagDistMeters = 4.0;
  public static final double maxMultiTagDistMeters = 4.0;

  // Stddev model: xyStdDev = max(floor, baseline * avgTagDist^2 / tagCount^2 * cameraFactor).
  // Lower baseline/floor = more trust = faster re-convergence after a collision, at the cost
  // of more jitter when tags are marginal. MT2 needs more trust than 6328's 0.01 coefficient
  // (tuned for their own solver). Tune on a known field position.
  public static final double xyStdDevBaseline = 0.08;
  public static final double xyStdDevFloor = 0.03;

  // Vision yaw is never trusted (MT2 yaw derives from the gyro we seeded it with)
  public static final double thetaStdDev = 9999999.0;

  // Limelight IMU modes (SetIMUMode)
  public static final int imuModeDisabled =
      1; // EXTERNAL_SEED: internal IMU stays seeded while disabled
  public static final int imuModeEnabledLL3 = 0; // EXTERNAL_ONLY
  public static final int imuModeEnabledLL4 = 4; // INTERNAL_EXTERNAL_ASSIST (LL4 hardware only)
}
