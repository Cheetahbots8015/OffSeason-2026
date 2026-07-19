package frc.robot.commands.ShooterCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;

public class ShootOnMoveDefaultCommand extends ShootOnMoveAimCommand {
  // Kept only for constructor compatibility with RobotContainer; not used in default aiming.
  private final HoodSubsystem hood;

  public ShootOnMoveDefaultCommand(
      Drive drive, TurretSubsystem turret, HoodSubsystem hood, ShootOnMoveCalculator calculator) {
    super(drive, turret, calculator);
    this.hood = hood;
  }

  @Override
  protected void onAim(AimResult result) {
    // Default command only aims the turret; hood control is handled by trigger commands.
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
