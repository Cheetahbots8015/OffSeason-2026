package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim motorSim;
  private double FeederAppliedVolts = 0.0;

  public FeederIOSim() {
    motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

    // Update simulation state
    motorSim.setInputVoltage(MathUtil.clamp(FeederAppliedVolts, -12.0, 12.0));
    motorSim.update(0.02);

    // Update motor inputs
    inputs.FeederPositionRad = motorSim.getAngularPositionRad();
    inputs.FeederVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.FeederAppliedVolts = motorSim.getInputVoltage();
    inputs.FeederCurrentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setFeederVoltage(double volts) {
    FeederAppliedVolts = volts;
  }
}
