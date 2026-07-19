package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.ShootOnMoveConstants;
import frc.robot.constants.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Shoot-on-the-move calculator.
 *
 * <p>Given the robot's field-relative pose and velocity, this class computes where the target
 * effectively was when the projectile arrives, then returns the turret angle and flywheel speed
 * needed to hit it. The solver iteratively refines the estimate because shot time depends on
 * distance, and distance depends on the effective target.
 */
public class ShootOnMoveCalculator {

  private final ShotProfile regularProfile = new ShotProfile();
  private final ShotProfile farProfile = new ShotProfile();

  private final Translation2d redTarget;
  private final Translation2d blueTarget;

  public ShootOnMoveCalculator(Translation2d redTarget, Translation2d blueTarget) {
    this.redTarget = redTarget;
    this.blueTarget = blueTarget;
    populateTables();
  }

  /**
   * Pre-populates the distance-to-flywheel-speed lookup table and fits the projectile-speed linear
   * regression. File data overrides the regression fallback; if the file is missing or empty, the
   * existing sqrt-curve regression is used instead.
   */
  private void populateTables() {
    List<double[]> calibrationData =
        regularProfile.loadFromFile("shoot-on-move-data.csv", "Calibration");
    List<double[]> farCalibrationData =
        farProfile.loadFromFile("shoot-on-move-far-data.csv", "FarCalibration");

    if (calibrationData.isEmpty()) {
      regularProfile.generateFallbackData(
          ShootOnMoveConstants.kFlywheelCurveSlope,
          ShootOnMoveConstants.kFlywheelCurveIntercept,
          ShooterConstants.kGear,
          ShooterConstants.kRadius);
    }

    if (farCalibrationData.isEmpty()) {
      farProfile.generateFallbackData(
          ShootOnMoveConstants.kFlywheelCurveSlope,
          ShootOnMoveConstants.kFlywheelCurveIntercept,
          ShooterConstants.kGear,
          ShooterConstants.kRadius);
    }

    List<Double> flywheelRpsValues = new ArrayList<>();
    List<Double> projectileSpeedValues = new ArrayList<>();
    for (double[] row : calibrationData) {
      flywheelRpsValues.add(row[1]);
      projectileSpeedValues.add(row[2]);
    }
    regularProfile.fitRegression(flywheelRpsValues, projectileSpeedValues);

    List<Double> farFlywheelRpsValues = new ArrayList<>();
    List<Double> farProjectileSpeedValues = new ArrayList<>();
    for (double[] row : farCalibrationData) {
      farFlywheelRpsValues.add(row[1]);
      farProjectileSpeedValues.add(row[2]);
    }
    farProfile.fitRegression(farFlywheelRpsValues, farProjectileSpeedValues);

    Logger.recordOutput("ShootOnMove/ProjectileSpeedSlope", regularProfile.getSlope());
    Logger.recordOutput("ShootOnMove/ProjectileSpeedIntercept", regularProfile.getIntercept());
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedSlope", farProfile.getSlope());
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedIntercept", farProfile.getIntercept());
  }

  /** Returns the current alliance target. */
  public Translation2d getTarget() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return redTarget;
    }
    return blueTarget;
  }

  /**
   * Computes shot parameters for shooting on the move.
   *
   * @param robotPose current field-relative pose
   * @param chassisSpeeds current field-relative chassis speeds (m/s and rad/s)
   * @return shot parameters including effective target, turret angle, and flywheel speed
   */
  public ShotParameters calculate(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    Translation2d robotPos = robotPose.getTranslation();
    Rotation2d robotHeading = robotPose.getRotation();
    Translation2d target = getTarget();
    Translation2d effectiveTarget = target;
    Translation2d fareffectiveTarget = target;
    double distance;
    double fardistance;

    for (int i = 0; i < ShootOnMoveConstants.kConvergenceIterations; i++) {
      distance = effectiveTarget.getDistance(robotPos);
      fardistance = fareffectiveTarget.getDistance(robotPos);

      double flywheelSpeed = regularProfile.getFlywheelSpeed(distance);
      double farflywheelSpeed = farProfile.getFlywheelSpeed(distance);

      double projectileSpeed =
          regularProfile.getProjectileSpeed(
              flywheelSpeed, ShootOnMoveConstants.hoodDefaultPosition);
      double farprojectileSpeed =
          farProfile.getProjectileSpeed(
              farflywheelSpeed,
              ShootOnMoveConstants.hoodDefaultPosition - ShootOnMoveConstants.hoodPositionOffset);

      double shotTime = distance / projectileSpeed;
      double farshotTime = fardistance / farprojectileSpeed;
      Logger.recordOutput("ShootOnMove/distance", distance);
      Logger.recordOutput("ShootOnMove/shotTime", shotTime);
      Logger.recordOutput("ShootOnMove/farShotTime", farshotTime);

      // Velocity of the shooter exit point due to chassis translation.
      Translation2d turretVelocity =
          new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

      // Chassis rotation contribution: rotate the robot-frame turret offset into field frame
      // and compute omega x r.
      double robotOmega = chassisSpeeds.omegaRadiansPerSecond;
      Translation2d turretOffsetField =
          ShootOnMoveConstants.kTurretOffsetMeters.rotateBy(robotHeading);
      double chassisRotVx = -robotOmega * turretOffsetField.getY();
      double chassisRotVy = robotOmega * turretOffsetField.getX();
      turretVelocity = turretVelocity.plus(new Translation2d(chassisRotVx, chassisRotVy));

      // Back-calculate the effective target location at the moment the shot was released.
      effectiveTarget = target.minus(turretVelocity.times(shotTime));
      fareffectiveTarget = target.minus(turretVelocity.times(farshotTime));
    }

    double finalDistance = effectiveTarget.getDistance(robotPos);
    double farFinalDistance = fareffectiveTarget.getDistance(robotPos);
    double flywheelSpeed;
    double farflywheelSpeed;
    if (ShootOnMoveConstants.useRegression) {
      flywheelSpeed = -0.0484 * finalDistance * finalDistance + 6.5589 * finalDistance + 26.142;
      farflywheelSpeed =
          0.9565 * farFinalDistance * farFinalDistance - 5.6649 * farFinalDistance + 51.237;
    } else {
      flywheelSpeed = regularProfile.getFlywheelSpeed(finalDistance);
      farflywheelSpeed = farProfile.getFlywheelSpeed(farFinalDistance);
    }
    // Aim from the turret's actual field position, not the robot center.
    Translation2d turretFieldPos =
        robotPos.plus(ShootOnMoveConstants.kTurretOffsetMeters.rotateBy(robotHeading));
    Rotation2d turretAngle = effectiveTarget.minus(turretFieldPos).getAngle();

    return new ShotParameters(effectiveTarget, turretAngle, flywheelSpeed, farflywheelSpeed);
  }

  /** Holds the result of a shoot-on-the-move calculation. */
  public static class ShotParameters {
    public final Translation2d effectiveTarget;
    public final Rotation2d turretAngle;
    public final double flywheelSpeedRps;
    public final double farFlyWheelSpeedRps;

    public ShotParameters(
        Translation2d effectiveTarget,
        Rotation2d turretAngle,
        double flywheelSpeedRps,
        double farFlywheelSpeedRps) {
      this.effectiveTarget = effectiveTarget;
      this.turretAngle = turretAngle;
      this.flywheelSpeedRps = flywheelSpeedRps;
      this.farFlyWheelSpeedRps = farFlywheelSpeedRps;
    }
  }
}
