package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretPositionVoltage extends Command {

  private final TurretSubsystem m_subsystem;
  private double m_degrees;

  public TurretPositionVoltage(TurretSubsystem subsystem, double degrees) {
    m_subsystem = subsystem;
    m_degrees = degrees;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setPosition(m_degrees);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
