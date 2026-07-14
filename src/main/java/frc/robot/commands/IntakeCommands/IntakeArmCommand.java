package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.CheetahUtil;

public class IntakeArmCommand extends Command {

  private final IntakeSubsystem m_subsystem;
  private double m_radians;

  public IntakeArmCommand(IntakeSubsystem subsystem, double radians) {
    m_subsystem = subsystem;
    m_radians = radians;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setArmPosition(m_radians);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return CheetahUtil.isNear(m_subsystem.getInput().ArmPositionRad, m_radians, 5);
  }
}
