package frc.robot.commands.IndexerCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexerVelocityVoltageCommand extends Command {

  private final IndexerSubsystem m_subsystem;
  private Debouncer jam_debouncer;
  private CommandXboxController m_controller;

  public IndexerVelocityVoltageCommand(
      IndexerSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    jam_debouncer = new Debouncer(0.5, DebounceType.kRising);
    jam_debouncer.calculate(false);
    SmartDashboard.putBoolean("Rumble", false);
    m_controller.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void execute() {
    boolean isJammed =
        jam_debouncer.calculate(m_subsystem.getInput().verticleVelocityRadPerSec < 280);
    m_subsystem.VelocityVoltage(
        isJammed ? 0 : SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
        SmartDashboard.getNumber("indexerVerticleVelocity", 0));
    // uses a debouncer, but why magic number 280?
    if (isJammed) {
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
