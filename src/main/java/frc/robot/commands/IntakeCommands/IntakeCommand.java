package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public IntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setFlywheelVoltage(3);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setFlywheelVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
