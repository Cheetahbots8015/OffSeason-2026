package frc.robot.commands.IndexerCommands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerVelocityVoltageCommand extends Command {

  private final IndexerSubsystem m_subsystem;
  private MedianFilter m_filter;
  private CommandXboxController m_controller;

  public IndexerVelocityVoltageCommand(
      IndexerSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_filter = new MedianFilter(20);
    for (int i = 0; i < 10; i++) {
      m_filter.calculate(250);
    }
    SmartDashboard.putBoolean("Rumble", false);
    m_controller.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void execute() {
    m_subsystem.VelocityVoltage(
        SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
        SmartDashboard.getNumber("indexerVerticleVelocity", 0));
    if (m_filter.calculate(m_subsystem.getInput().verticleVelocityRadPerSec) < 280) {
      m_controller.setRumble(RumbleType.kBothRumble, 1);
      SmartDashboard.putBoolean("Rumble", true);
    } else {
      m_controller.setRumble(RumbleType.kBothRumble, 0);
      SmartDashboard.putBoolean("Rumble", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorVoltage(0, 0);
    m_controller.setRumble(RumbleType.kBothRumble, 0);
    SmartDashboard.putBoolean("Rumble", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
