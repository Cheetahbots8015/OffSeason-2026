package frc.robot.constants;

public final class HoodConstants {
  private HoodConstants() {}

  // Motor and sensor IDs
  public static final int kHoodMotorID = 34;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = false;
  public static final boolean kMotorInvertCCWPositive = true;

  // Control tuning
  public static final double kStatusUpdateFrequency = 50.0;
  public static final double kSlot_kP = 5;
  public static final double kSlot_kI = 0;
  public static final double kSlot_kD = 0;
  public static final double kSlot_kS = 0.35;
  public static final double kSlot_kV = 0.2;

  public static final double gearRatio = 1 / 40.625;
}
