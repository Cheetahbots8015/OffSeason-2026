package frc.robot.constants;

public class IndexerConstants {
  private IndexerConstants() {}

  public static final int kHoriMotorID = 26;
  public static final int kVertMotorID = 27;

  public static final boolean kHoriNeutralCoast = true;
  public static final boolean kVertNeutralCoast = true;

  public static final boolean kHoriInvert = true;
  public static final boolean kVertInvert = true;

  public static final double kPv = 0.6;
  public static final double kIv = 0.005;
  public static final double kDv = 0.0;
  public static final double kVv = 0.2;

  public static final double kPh = 0.6;
  public static final double kIh = 0.005;
  public static final double kDh = 0.0;
  public static final double kVh = 0.2;

  public static final double kStatusUpdateFrequency = 50.0;
}
