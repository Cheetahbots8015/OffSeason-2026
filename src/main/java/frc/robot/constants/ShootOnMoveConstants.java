package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class ShootOnMoveConstants {
  private ShootOnMoveConstants() {}

  /** Red alliance target (center of the goal / scoring location). */
  public static final Translation2d kRedTarget =
      new Translation2d((12.519177399999998 + 11.3118646) / 2, 4.0346376);

  /** Blue alliance target (center of the goal / scoring location). */
  public static final Translation2d kBlueTarget =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376);

  /** Turret shooter exit point relative to the robot center of rotation (meters). */
  public static final Translation2d kTurretOffsetMeters = new Translation2d(0.122, -0.122);

  /** Number of iterations for the effective-target convergence solver. */
  public static final int kConvergenceIterations = 5;

  /**
   * Flywheel curve: PredictedVelocity = sqrt(distance * kFlywheelCurveSlope +
   * kFlywheelCurveIntercept)
   */
  public static final double kFlywheelCurveSlope = 14475.0;

  public static final double kFlywheelCurveIntercept = 56439.0;

  /** Distance (meters) subtracted from the target distance in the existing ShootCommand. */
  // TODO: Should be deleted using limelight
  public static final double kTargetDistanceOffsetMeters = 0.6036;

  /** Distance (meters) at which to switch to the far shot regression. */
  public static final double switchDistanceMeters = 5.0;

  /** Offset (meters) to apply to the hood position on far shot when shooting on the move. */
  // TODO: Replcae the placeholder
  public static final double hoodPositionOffset = 0.0;
}
