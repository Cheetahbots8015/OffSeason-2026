package frc.robot.constants;

public class IndexerConstants {
  private IndexerConstants() {}

  public static final int kHoriMotorID = 26;
  public static final int kVertMotorID = 27;

  public static final boolean kHoriNeutralCoast = true;
  public static final boolean kVertNeutralCoast = true;

  public static final boolean kHoriInvert = true;
  public static final boolean kVertInvert = true;

  public static final double kP = 0.6;
  public static final double kI = 0.005;
  public static final double kD = 0.0;
  public static final double kV = 0.3;

  public static final double kStatusUpdateFrequency = 50.0;
}
