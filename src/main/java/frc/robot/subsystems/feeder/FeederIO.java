package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double FeederPositionRad = 0.0;
    public double FeederVelocityRotPerSec = 0.0;
    public double FeederAppliedVolts = 0.0;
    public double FeederCurrentAmps = 0.0;
  }

  public default void setOpenLoop(double motorOutput) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeederVoltage(double volts) {}

  public default void setFeederVelocityVoltage(double velocity) {}
}
