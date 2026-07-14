package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeAutoCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private Timer m_timer;
  private double m_time;

  public IntakeAutoCommand(IntakeSubsystem subsystem, double time) {
    m_subsystem = subsystem;
    m_time = time;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    m_subsystem.setArmPosition(-75);
    m_subsystem.setFlywheelVoltage(6);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setFlywheelVoltage(0);
    m_subsystem.setArmPosition(0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
