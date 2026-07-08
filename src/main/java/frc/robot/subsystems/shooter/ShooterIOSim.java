package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim rightSim;

  private double rightAppliedVolts = 0.0;

  public ShooterIOSim() {
    rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    rightSim.setInputVoltage(MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    rightSim.update(0.02);

    inputs.rightPositionRad = rightSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec =
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec());
    inputs.rightAppliedVolts = rightSim.getInputVoltage();
    inputs.rightCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
  }

  @Override
  public void setMotorVoltage(double volts) {
    rightAppliedVolts = volts;
  }
}
