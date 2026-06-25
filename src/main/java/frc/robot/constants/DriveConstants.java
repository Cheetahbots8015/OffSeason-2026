package frc.robot.constants;

public class DriveConstants {
  public static final double robotMassKg = 58;
  public static final double robotMOI = 4.084;
  public static final double wheelCOF = 1.2;
  // PID constants for path following
  public static final double autoTranslationkP = 5.0;
  public static final double autoTranslationkI = 0.0;
  public static final double autoTranslationkD = 0.0;
  public static final double autoRotationkP = 5.0;
  public static final double autoRotationkI = 0.0;
  public static final double autoRotationkD = 0.0;
  public static final double statusUpdateFrequency = 50.0;
  // Simulation PID constants
  public static final double simDriveKp = 0.05;
  public static final double simDriveKd = 0.0;
  public static final double simDriveKs = 0.0;
  public static final double simDriveKvRot = 0.91035;
  public static final double simTurnKp = 8.0;
  public static final double simTurnKd = 0.0;

  public static final int[] redTags =
      new int[] {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  public static final int[] blueTags =
      new int[] {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};

  public static final double maxAmbiguity = 999;
  public static final double maxCameraDist = 2.0;
  public static final double minArea = 0.0;
}
