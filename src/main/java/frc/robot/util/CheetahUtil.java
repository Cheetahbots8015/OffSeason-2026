package frc.robot.util;

import frc.robot.constants.TurretConstants;
import java.util.function.DoubleSupplier;

public class CheetahUtil {

  // --- General Math Utilities ---

  /** Checks if a value is within a specified tolerance of a target value. */
  public static boolean isNear(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }

  /** Checks if a value is near a target value using a default tolerance of 0.05. */
  public static boolean isNear(double value, double target) {
    return isNear(value, target, 0.05);
  }

  /** Checks if a value is within a percentage-based tolerance of a target value. */
  public static boolean isNearPercentage(double value, double target, double percentage) {
    double tolerance = Math.abs(target * percentage);
    return isNear(value, target, tolerance);
  }

  /** Checks if a value is near a target value using a default percentage tolerance of 1%. */
  public static boolean isNearPercentage(double value, double target) {
    return isNearPercentage(value, target, 0.01);
  }

  /** Applies a simple deadband. Returns 0.0 if the value is within the deadband. */
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }

  /** Applies a deadband to a value provided by a DoubleSupplier. */
  public static double applyDeadband(DoubleSupplier valueSupplier, double deadband) {
    double value = valueSupplier.getAsDouble();
    return applyDeadband(value, deadband);
  }

  // --- Elevator Unit Conversions ---

  /**
   * Converts elevator motor rotations to height in meters. Formula: rotations / gear_ratio *
   * (distance_per_rotation) + offset
   */
  public static double elevatorRotationToMeters(double rotations) {
    return rotations / 6.0 * (0.005 * 30) + 0.362;
  }

  /** Converts height in meters to elevator motor rotations. */
  public static double elevatorMetersToRotation(double meters) {
    return (meters - 0.362) * 6.0 / (0.005 * 30);
  }

  public static double turretRotationsToDeg(double rotations) {
    return rotations * TurretConstants.gearRatio * 360.0;
  }

  public static double turretDegToRotations(double degrees) {
    return degrees / TurretConstants.gearRatio / 360.0;
  }
}
