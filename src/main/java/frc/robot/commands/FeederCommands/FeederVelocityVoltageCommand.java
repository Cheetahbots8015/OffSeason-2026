package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class FeederVelocityVoltageCommand extends Command {

  private final FeederSubsystem m_subsystem;

  public FeederVelocityVoltageCommand(FeederSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
