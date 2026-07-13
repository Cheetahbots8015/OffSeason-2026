package frc.robot.constants;

public final class ShooterConstants {
  private ShooterConstants() {}

  // CAN IDs
  public static final int kLeftMotorID = 30;
  public static final int kRightMotorID = 29;

  // Motor configuration
  public static final boolean kLeftNeutralCoast = true;
  public static final boolean kLeftInvert = false;
  public static final boolean kRightNeutralCoast = true;
  public static final boolean kRightInvert = true;

  // Control slots / tuning
  public static final double kStatusUpdateFrequency = 50.0;

  public static final double kP = 0.05;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kA = 0;
  public static final double kS = 0.0;
  public static final double kV = 0.13;

  // Gear ratio
  // TODO: update gear ratio
  public static final double kGear = 1.0;
  public static final double kRadius = 0.05;
}
