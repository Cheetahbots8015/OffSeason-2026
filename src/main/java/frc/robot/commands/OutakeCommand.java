package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class OutakeCommand extends Command {

  private final IndexerSubsystem m_indexer;
  private final FeederSubsystem m_feeder;

  public OutakeCommand(IndexerSubsystem indexer, FeederSubsystem feeder) {
    m_indexer = indexer;
    m_feeder = feeder;
    addRequirements(indexer, feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_indexer.VelocityVoltage(-250, -250);
    m_feeder.setFeederVelocityVoltage(-300);
  }

  @Override
  public void end(boolean interrupted) {
    m_indexer.setMotorVoltage(0, 0);
    m_feeder.setFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
