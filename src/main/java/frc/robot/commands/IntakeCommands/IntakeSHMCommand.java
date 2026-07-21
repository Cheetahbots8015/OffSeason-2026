package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.CheetahUtil;

public class IntakeSHMCommand extends Command {

  private final IntakeSubsystem m_subsystem;

  public IntakeSHMCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.setFlywheelVoltage(3);
    if (CheetahUtil.isNear(m_subsystem.getInput().ArmPositionRad, -50, 12.5)) {
      m_subsystem.setArmPosition(-75);
    } else {
      m_subsystem.setArmPosition(-50);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setFlywheelVoltage(0);
    m_subsystem.setArmPosition(-75);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
