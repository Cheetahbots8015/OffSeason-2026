package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double horizontalPositionRad = 0.0;
    public double horizontalVelocityRadPerSec = 0.0;
    public double horizontalAppliedVolts = 0.0;
    public double horizontalCurrentAmps = 0.0;
    public double verticlePositionRad = 0.0;
    public double verticleVelocityRadPerSec = 0.0;
    public double verticleAppliedVolts = 0.0;
    public double verticleCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Direct set motor voltage - single motor (applied to both/follower). */
  public default void setMotorVoltage(double horivolts, double vertvolts) {}
}
