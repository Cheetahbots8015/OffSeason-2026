package frc.robot.commands.HoodCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class HoodCommand extends Command {

  private final HoodSubsystem m_subsystem;
  private double m_volts;

  public HoodCommand(HoodSubsystem subsystem, double volts) {
    m_subsystem = subsystem;
    m_volts = volts;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setMotorVoltage(m_volts);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
