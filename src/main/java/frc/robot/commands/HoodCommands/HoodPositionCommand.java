package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class HoodPositionCommand extends Command {

  private final HoodSubsystem m_subsystem;
  private double m_deg;

  public HoodPositionCommand(HoodSubsystem subsystem, double degrees) {
    m_subsystem = subsystem;
    m_deg = degrees;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(m_deg);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
