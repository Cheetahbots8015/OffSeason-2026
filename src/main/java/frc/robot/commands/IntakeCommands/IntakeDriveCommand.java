package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeDriveCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private double driveVolt;

  public IntakeDriveCommand(IntakeSubsystem subsystem, double volts) {
    m_subsystem = subsystem;
    driveVolt = volts;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setArmVoltage(driveVolt);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setArmVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
