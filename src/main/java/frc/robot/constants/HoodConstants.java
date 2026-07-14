package frc.robot.constants;

public final class HoodConstants {
  private HoodConstants() {}

  // Motor and sensor IDs
  public static final int kHoodMotorID = 34;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = false;
  public static final boolean kMotorInvertCCWPositive = false;

  // Control tuning
  public static final double kStatusUpdateFrequency = 50.0;
  public static final double kSlot_kP = 0;
  public static final double kSlot_kI = 0;
  public static final double kSlot_kD = 0;
  public static final double kSlot_kS = 0;
  public static final double kSlot_kV = 0;

  // TODO: Replace the placeholder
  public static final double gearRatio = 1;
}
