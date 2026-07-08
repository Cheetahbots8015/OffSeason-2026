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

  public static final double kLeftSlot_kP = 0.0;
  public static final double kLeftSlot_kI = 0.0;
  public static final double kLeftSlot_kD = 0.0;
  public static final double kLeftSlot_kA = 0.0032085;
  public static final double kLeftSlot_kS = 0.0;
  public static final double kLeftSlot_kV = 0.020658;

  public static final double kRightSlot_kP = 0.0;
  public static final double kRightSlot_kI = 0.0;
  public static final double kRightSlot_kD = 0.0;
  public static final double kRightSlot_kA = 0.0032085;
  public static final double kRightSlot_kS = 0.0;
  public static final double kRightSlot_kV = 0.020658;
}
