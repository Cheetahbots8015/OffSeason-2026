package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.CheetahUtil;

public class HoodIOSim implements HoodIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSim;
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    inputs.motorPositionDeg = Math.toDegrees(motorSim.getAngularPositionRad());
    inputs.motorVelocityRotPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.motorAppliedVolts = motorSim.getInputVoltage();
    inputs.motorCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.hoodPositionDeg =
        CheetahUtil.hoodRotationsToDeg(motorSim.getAngularPositionRad() / (2.0 * Math.PI));
  }

  @Override
  public void setMotorVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double positionDeg) {
    // naive: set applied volts proportional to position error (very simple)
    double error = Math.toRadians(positionDeg) - motorSim.getAngularPositionRad();
    appliedVolts = Math.max(-12.0, Math.min(12.0, 6.0 * error));
  }
}
