package frc.robot.subsystems.indexer;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.FinanceDepartment;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(0.5).per(Units.Second),
                Units.Volts.of(5.0),
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt), 0), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    // Report energy usage (supply current = battery draw)
    FinanceDepartment.getInstance()
        .reportCurrentUsage(
            "Indexer", false, inputs.horizontalSupplyCurrentAmps, inputs.verticleSupplyCurrentAmps);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void setMotorVoltage(double horivolts, double vertvolts) {
    io.setMotorVoltage(horivolts, vertvolts);
  }

  public void VelocityVoltage(double horiradians, double vertradians) {
    io.VelocityVoltage(horiradians, vertradians);
  }

  public IndexerIOInputs getInput() {
    return inputs;
  }
}
