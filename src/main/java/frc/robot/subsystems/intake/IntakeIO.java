package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double FlywheelPositionRad = 0.0;
    public double FlywheelVelocityRadPerSec = 0.0;
    public double FlywheelAppliedVolts = 0.0;
    public double FlywheelCurrentAmps = 0.0;
    public double FlywheelSupplyCurrentAmps = 0.0;

    public double FollowerPositionRad = 0.0;
    public double FollowerVelocityRadPerSec = 0.0;
    public double FollowerAppliedVolts = 0.0;
    public double FollowerCurrentAmps = 0.0;
    public double FollowerSupplyCurrentAmps = 0.0;

    public double ArmPositionRad = 0.0;
    public double ArmVelocityRadPerSec = 0.0;
    public double ArmAppliedVolts = 0.0;
    public double ArmCurrentAmps = 0.0;
    public double ArmSupplyCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setFlywheelVoltage(double volts) {}

  public default void setArmVoltage(double volts) {}

  public default void setArmPosition(double radians) {}
}
