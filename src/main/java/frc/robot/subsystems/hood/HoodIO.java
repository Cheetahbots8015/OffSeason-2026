package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    // Motor
    public double motorPosition = 0.0;
    public double motorVelocityRotPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;

    public double hoodPositionDeg = 0.0;
  }

  /** Update inputs for logging and state. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Directly set motor voltage (volts) */
  public default void setMotorVoltage(double volts) {}

  /** Position control (MotionMagic) - angle in radians */
  public default void setPosition(double positionRad) {}

  /** Set hood neutral state */
  public default void setHoodNeutral(boolean neutral) {}
}
