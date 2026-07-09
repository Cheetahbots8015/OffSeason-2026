package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretCommand extends Command {

  private final TurretSubsystem m_subsystem;
  private double m_volts;

  public TurretCommand(TurretSubsystem subsystem, double volts) {
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
