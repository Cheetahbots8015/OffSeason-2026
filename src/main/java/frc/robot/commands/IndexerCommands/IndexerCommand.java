package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerCommand extends Command {

  private final IndexerSubsystem m_subsystem;

  public IndexerCommand(IndexerSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setMotorVoltage(
        SmartDashboard.getNumber("indexerHorizontalVolt", 0),
        SmartDashboard.getNumber("indexerVerticleVolt", 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorVoltage(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
