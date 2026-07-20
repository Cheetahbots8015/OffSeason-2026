package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    // Motor
    public double motorPositionDeg = 0.0;
    public double motorVelocityRotPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrentAmps = 0.0;
    public double motorSupplyCurrentAmps = 0.0;

    public double turretPositionDeg = 0.0;

    public boolean turretLocked = false;
  }

  /** Update inputs for logging and state. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Directly set motor voltage (volts) */
  public default void setMotorVoltage(double volts) {}

  /** Position control (MotionMagic/PositionVoltage) - angle in radians */
  public default void setPosition(double positionRad) {}

  public default void lockTurret(boolean lock) {}
}
