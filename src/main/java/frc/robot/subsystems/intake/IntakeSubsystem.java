// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.FinanceDepartment;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setArmVoltage(voltage.in(Units.Volt)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Report energy usage (supply current = battery draw)
    FinanceDepartment.getInstance()
        .reportCurrentUsage(
            "Intake/Flywheel",
            false,
            inputs.FlywheelSupplyCurrentAmps,
            inputs.FollowerSupplyCurrentAmps);
    FinanceDepartment.getInstance()
        .reportCurrentUsage("Intake/Arm", false, inputs.ArmSupplyCurrentAmps);
  }

  public void setArmVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setFlywheelVoltage(double volts) {
    io.setFlywheelVoltage(volts);
  }

  public void setArmPosition(double radians) {
    io.setArmPosition(radians);
  }

  public IntakeIOInputs getInput() {
    return inputs;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
