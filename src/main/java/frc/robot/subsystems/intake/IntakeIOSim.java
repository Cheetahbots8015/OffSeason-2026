package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim IntakeSim;
  private final DCMotorSim ArmSim;
  private double IntakeAppliedVolts = 0.0;
  private double ArmAppliedVolts = 0.0;

  public IntakeIOSim() {
    IntakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
    ArmSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 0.001, 1), GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Update simulation state
    IntakeSim.setInputVoltage(MathUtil.clamp(IntakeAppliedVolts, -12.0, 12.0));
    IntakeSim.update(0.02);
    ArmSim.setInputVoltage(MathUtil.clamp(ArmAppliedVolts, -12.0, 12.0));
    ArmSim.update(0.02);

    // Update motor inputs
    inputs.FlywheelPositionRad = IntakeSim.getAngularPositionRad();
    inputs.FlywheelVelocityRadPerSec = IntakeSim.getAngularVelocityRadPerSec();
    inputs.FlywheelAppliedVolts = IntakeSim.getInputVoltage();
    inputs.FlywheelCurrentAmps = Math.abs(IntakeSim.getCurrentDrawAmps());

    inputs.ArmPositionRad = ArmSim.getAngularPositionRad();
    inputs.ArmVelocityRadPerSec = ArmSim.getAngularVelocityRadPerSec();
    inputs.ArmAppliedVolts = IntakeSim.getInputVoltage();
    inputs.ArmCurrentAmps = Math.abs(ArmSim.getCurrentDrawAmps());
  }
}
