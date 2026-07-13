package frc.robot.constants;

public final class TurretConstants {
  private TurretConstants() {}

  // Motor and sensor IDs
  public static final int kTurretMotorID = 33;
  public static final int kPigeonId = 35;

  // Motor configuration
  public static final boolean kMotorNeutralCoast = false;
  public static final boolean kMotorInvertCCWPositive = false;

  // Control tuning
  public static final double kStatusUpdateFrequency = 50.0;
  public static final double kSlot_kP = 2;
  public static final double kSlot_kI = 0; // 0?
  public static final double kSlot_kD = 0.0;
  public static final double kSlot_kS = 0;
  public static final double kSlot_kV = 0.15;

  public static final double gearRatio = 1.0 / 12;
}
