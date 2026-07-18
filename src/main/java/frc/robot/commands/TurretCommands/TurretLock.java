package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurretLock extends Command {

  private final TurretSubsystem m_subsystem;
  private boolean m_locked;

  public TurretLock(TurretSubsystem subsystem, boolean locked) {
    m_subsystem = subsystem;
    m_locked = locked;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.lockTurret(m_locked);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
