package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class HoodNeutral extends Command {

  private final HoodSubsystem m_subsystem;
  private boolean m_neutral;

  public HoodNeutral(HoodSubsystem subsystem, boolean neutral) {
    m_subsystem = subsystem;
    m_neutral = neutral;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setHoodNeutral(m_neutral);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
