package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.ShootOnMoveConstants;
import frc.robot.constants.ShooterConstants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
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

  // Distance (meters) -> flywheel speed (rotations per second)
  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeed =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap farDistanceToFlywheelSpeed =
      new InterpolatingDoubleTreeMap();

  // Linear model: projectileSpeedMps = m * flywheelRps + b
  private double projectileSpeedSlope;
  private double projectileSpeedIntercept;
  private double farProjectileSpeedSlope;
  private double farProjectileSpeedIntercept;

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
    List<double[]> calibrationData = loadCalibrationDataFromFile();
    List<double[]> farCalibrationData = loadFarCalibrationDataFromFile();

    // If no calibration data was loaded from file, fall back to the regression estimate.
    if (calibrationData.isEmpty()) {
      for (double distanceMeters = 1.0; distanceMeters <= 8.0; distanceMeters += 0.5) {
        double flywheelRps =
            Math.sqrt(
                    distanceMeters * ShootOnMoveConstants.kFlywheelCurveSlope
                        + ShootOnMoveConstants.kFlywheelCurveIntercept)
                / (2.0 * Math.PI);
        double projectileSpeedMps =
            flywheelRps * ShooterConstants.kGear * ShooterConstants.kRadius * 2 * Math.PI;
        calibrationData.add(new double[] {distanceMeters, flywheelRps, projectileSpeedMps});
      }
    }
    if (farCalibrationData.isEmpty()) {
      for (double distanceMeters = 1.0; distanceMeters <= 8.0; distanceMeters += 0.5) {
        double flywheelRps =
            Math.sqrt(
                    distanceMeters * ShootOnMoveConstants.kFlywheelCurveSlope
                        + ShootOnMoveConstants.kFlywheelCurveIntercept)
                / (2.0 * Math.PI);
        double projectileSpeedMps =
            flywheelRps * ShooterConstants.kGear * ShooterConstants.kRadius * 2 * Math.PI;
        farCalibrationData.add(new double[] {distanceMeters, flywheelRps, projectileSpeedMps});
      }
    }

    List<Double> flywheelRpsValues = new ArrayList<>();
    List<Double> projectileSpeedValues = new ArrayList<>();
    for (double[] row : calibrationData) {
      distanceToFlywheelSpeed.put(row[0], row[1]);
      flywheelRpsValues.add(row[1]);
      projectileSpeedValues.add(row[2]);
    }
    fitProjectileSpeedRegression(flywheelRpsValues, projectileSpeedValues);

    List<Double> farFlywheelRpsValues = new ArrayList<>();
    List<Double> farProjectileSpeedValues = new ArrayList<>();
    for (double[] row : farCalibrationData) {
      farDistanceToFlywheelSpeed.put(row[0], row[1]);
      farFlywheelRpsValues.add(row[1]);
      farProjectileSpeedValues.add(row[2]);
    }
    farFitProjectileSpeedRegression(farFlywheelRpsValues, farProjectileSpeedValues);

    Logger.recordOutput("ShootOnMove/ProjectileSpeedSlope", projectileSpeedSlope);
    Logger.recordOutput("ShootOnMove/ProjectileSpeedIntercept", projectileSpeedIntercept);
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedSlope", farProjectileSpeedSlope);
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedIntercept", farProjectileSpeedIntercept);
  }

  /**
   * Loads measured calibration data from {@code deploy/shoot-on-move-data.csv}.
   *
   * <p>Expected columns: distanceMeters, flywheelRps, projectileSpeedMps. If the file is missing or
   * malformed, an empty list is returned.
   *
   * @return list of calibration triples {distanceMeters, flywheelRps, projectileSpeedMps}
   */
  private List<double[]> loadCalibrationDataFromFile() {
    List<double[]> data = new ArrayList<>();
    File file = new File(Filesystem.getDeployDirectory(), "shoot-on-move-data.csv");
    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
      String line;
      boolean firstLine = true;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) {
          continue;
        }
        if (firstLine) {
          firstLine = false;
          continue;
        }
        String[] parts = line.split(",");
        if (parts.length >= 3) {
          double distanceMeters = Double.parseDouble(parts[0].trim());
          double flywheelRps = Double.parseDouble(parts[1].trim());
          double projectileSpeedMps = Double.parseDouble(parts[2].trim());
          data.add(new double[] {distanceMeters, flywheelRps, projectileSpeedMps});
        }
      }
      Logger.recordOutput(
          "ShootOnMove/CalibrationDataLoaded", "Loaded " + data.size() + " entries from file");
    } catch (Exception e) {
      Logger.recordOutput(
          "ShootOnMove/CalibrationDataLoadError",
          "Failed to load " + file.getAbsolutePath() + ": " + e.getMessage());
    }
    return data;
  }

  private List<double[]> loadFarCalibrationDataFromFile() {
    List<double[]> data = new ArrayList<>();
    File file = new File(Filesystem.getDeployDirectory(), "shoot-on-move-far-data.csv");
    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
      String line;
      boolean firstLine = true;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) {
          continue;
        }
        if (firstLine) {
          firstLine = false;
          continue;
        }
        String[] parts = line.split(",");
        if (parts.length >= 3) {
          double distanceMeters = Double.parseDouble(parts[0].trim());
          double flywheelRps = Double.parseDouble(parts[1].trim());
          double projectileSpeedMps = Double.parseDouble(parts[2].trim());
          data.add(new double[] {distanceMeters, flywheelRps, projectileSpeedMps});
        }
      }
      Logger.recordOutput(
          "ShootOnMove/FarCalibrationDataLoaded", "Loaded " + data.size() + " entries from file");
    } catch (Exception e) {
      Logger.recordOutput(
          "ShootOnMove/FarCalibrationDataLoadError",
          "Failed to load " + file.getAbsolutePath() + ": " + e.getMessage());
    }
    return data;
  }

  /**
   * Fits a linear regression of projectile speed (m/s) vs flywheel speed (RPS).
   *
   * <p>Model: projectileSpeed = slope * flywheelRps + intercept
   */
  private void fitProjectileSpeedRegression(
      List<Double> flywheelRpsValues, List<Double> projectileSpeedValues) {
    int n = flywheelRpsValues.size();
    if (n < 2) {
      projectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius * Math.PI * 2.0;
      projectileSpeedIntercept = 0.0;
      return;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = flywheelRpsValues.get(i);
      double y = projectileSpeedValues.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }

    double denominator = n * sumX2 - sumX * sumX;
    if (Math.abs(denominator) < 1e-9) {
      projectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius;
      projectileSpeedIntercept = 0.0;
      return;
    }

    projectileSpeedSlope = (n * sumXY - sumX * sumY) / denominator;
    projectileSpeedIntercept = (sumY - projectileSpeedSlope * sumX) / n;
  }

  /**
   * Fits a linear regression of projectile speed (m/s) vs flywheel speed (RPS) for far shots.
   *
   * <p>Model: projectileSpeed = slope * flywheelRps + intercept
   */
  private void farFitProjectileSpeedRegression(
      List<Double> flywheelRpsValues, List<Double> projectileSpeedValues) {
    int n = flywheelRpsValues.size();
    if (n < 2) {
      farProjectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius * Math.PI * 2.0;
      farProjectileSpeedIntercept = 0.0;
      return;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = flywheelRpsValues.get(i);
      double y = projectileSpeedValues.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }

    double denominator = n * sumX2 - sumX * sumX;
    if (Math.abs(denominator) < 1e-9) {
      farProjectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius;
      farProjectileSpeedIntercept = 0.0;
      return;
    }

    farProjectileSpeedSlope = (n * sumXY - sumX * sumY) / denominator;
    farProjectileSpeedIntercept = (sumY - farProjectileSpeedSlope * sumX) / n;
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
    double distance = effectiveTarget.getDistance(robotPos);
    double fardistance = fareffectiveTarget.getDistance(robotPos);

    for (int i = 0; i < ShootOnMoveConstants.kConvergenceIterations; i++) {
      distance = effectiveTarget.getDistance(robotPos);
      fardistance = fareffectiveTarget.getDistance(robotPos);

      double flywheelSpeed = distanceToFlywheelSpeed.get(distance);
      double farflywheelSpeed = farDistanceToFlywheelSpeed.get(distance);

      double projectileSpeed = projectileSpeedSlope * flywheelSpeed + projectileSpeedIntercept;
      double farprojectileSpeed =
          farProjectileSpeedSlope * farflywheelSpeed + farProjectileSpeedIntercept;

      projectileSpeed *= Math.cos(Math.toRadians(ShootOnMoveConstants.hoodDefaultPosition));
      farprojectileSpeed *=
          Math.cos(
              Math.toRadians(
                  ShootOnMoveConstants.hoodDefaultPosition
                      - ShootOnMoveConstants.hoodPositionOffset));

      double shotTime = distance / projectileSpeed;
      double farshotTime = distance / farprojectileSpeed;
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
    double flywheelSpeed = distanceToFlywheelSpeed.get(finalDistance);
    double farflywheelSpeed = farDistanceToFlywheelSpeed.get(farFinalDistance);

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
