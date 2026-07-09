package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootVelocityVoltageCommand extends Command {

  private final ShooterSubsystem m_subsystem;

  public ShootVelocityVoltageCommand(ShooterSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.VelocityVoltage(SmartDashboard.getNumber("shooterVelocity", 0));
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
