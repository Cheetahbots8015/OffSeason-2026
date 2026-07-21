package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.CheetahUtil;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;

public class ShootOnMoveTriggerCommand extends ShootOnMoveAimCommand {
  protected final HoodSubsystem hood;
  protected final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  protected final FeederSubsystem feeder;

  public ShootOnMoveTriggerCommand(
      Drive drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShootOnMoveCalculator calculator) {
    super(drive, turret, calculator);
    this.hood = hood;
    this.shooter = shooter;
    this.indexer = indexer;
    this.feeder = feeder;
    addRequirements(hood, shooter, indexer, feeder);
  }

  @Override
  protected void onAim(AimResult result) {
    double turretSetpoint = result.turretSetpointDeg();
    ShotParameters params = result.parameters();

    double shooterSetpointRadPerSec = getFlywheelSpeedRps(params) * 2.0 * Math.PI;
    if (turret.getIo().turretLocked) {
      shooter.VelocityVoltage(250);
    } else {
      shooter.VelocityVoltage(
          shooterSetpointRadPerSec + SmartDashboard.getNumber("shooterOffset", 0));
    }
    hood.setPosition(getHoodPositionDeg());

    if (CheetahUtil.isNear(turret.getPosition(), turretSetpoint, 10)
        && CheetahUtil.isNear(
            shooterSetpointRadPerSec + SmartDashboard.getNumber("shooterOffset", 0),
            shooter.getMotorVelocity(),
            10)) {
      indexer.VelocityVoltage(
          SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
          SmartDashboard.getNumber("indexerVerticleVelocity", 0));
      feeder.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
    } else if(turret.getIo().turretLocked && CheetahUtil.isNear(shooter.getMotorVelocity(), 250, 10)){
      indexer.VelocityVoltage(
          SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
          SmartDashboard.getNumber("indexerVerticleVelocity", 0));
      feeder.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
    }
    else {
      indexer.setMotorVoltage(0, 0);
      feeder.setFeederVoltage(0);
    }
  }

  protected double getFlywheelSpeedRps(ShotParameters params) {
    return params.flywheelSpeedRps;
  }

  protected double getHoodPositionDeg() {
    return 0.0;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    onEndShooting();
  }

  protected void onEndShooting() {
    shooter.setMotorVoltage(0.0);
    indexer.setMotorVoltage(0, 0);
    feeder.setFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
