package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerVelocityVoltageCommand extends Command {

  private final IndexerSubsystem m_subsystem;

  public IndexerVelocityVoltageCommand(IndexerSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.VelocityVoltage(
        SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
        SmartDashboard.getNumber("indexerVerticleVelocity", 0));
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
