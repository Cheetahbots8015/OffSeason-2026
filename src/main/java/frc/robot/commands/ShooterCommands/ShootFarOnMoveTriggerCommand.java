package frc.robot.commands.ShooterCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;

public class ShootFarOnMoveTriggerCommand extends ShootOnMoveTriggerCommand {

  public ShootFarOnMoveTriggerCommand(
      Drive drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShootOnMoveCalculator calculator) {
    super(drive, turret, hood, shooter, indexer, feeder, calculator);
  }

  @Override
  protected double getFlywheelSpeedRps(ShotParameters params) {
    return params.farFlyWheelSpeedRps;
  }

  @Override
  protected double getHoodPositionDeg() {
    return 20.0;
  }

  @Override
  protected void onEndShooting() {
    super.onEndShooting();
    hood.setPosition(0);
  }
}
