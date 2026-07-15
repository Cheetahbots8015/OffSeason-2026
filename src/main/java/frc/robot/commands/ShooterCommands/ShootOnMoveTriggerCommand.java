package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.CheetahUtil;
import frc.robot.util.ShootOnMoveCalculator;
import org.littletonrobotics.junction.Logger;

/**
 * Aims the turret and spins the flywheel to shoot at the alliance target while the robot is moving.
 *
 * <p>The driver retains full control of chassis translation and rotation. This command reads the
 * robot's field-relative pose and chassis speeds, runs the shoot-on-the-move solver, and commands
 * the turret and shooter.
 */
public class ShootOnMoveTriggerCommand extends Command {
  private final Drive drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final FeederSubsystem feeder;
  private final ShootOnMoveCalculator calculator;

  public ShootOnMoveTriggerCommand(
      Drive drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShootOnMoveCalculator calculator) {
    this.drive = drive;
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.indexer = indexer;
    this.feeder = feeder;
    this.calculator = calculator;
    addRequirements(turret, shooter, indexer, feeder, hood);
  }

  @Override
  public void execute() {
    // Drive returns robot-relative chassis speeds; the SOTM solver needs field-relative speeds.
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, drive.getRotation());
    ShootOnMoveCalculator.ShotParameters params =
        calculator.calculate(drive.getPose(), fieldRelativeSpeeds);

    // Convert field-relative aim angle to turret-relative degrees.
    Rotation2d robotHeading = drive.getRotation();
    double turretSetpointDeg = params.turretAngle.minus(robotHeading).getDegrees();

    double shooterSetpointRadPerSec = params.flywheelSpeedRps * 2.0 * Math.PI;

    double turretOffset = 60;
    shooter.VelocityVoltage(shooterSetpointRadPerSec);

    double turretSetpoint = wrapTo180(turretSetpointDeg + turretOffset);
    if (turretSetpoint > 170) {
      turret.setPosition(170);
    } else if (turretSetpoint < -170) {
      turret.setPosition(-170);
    } else {
      turret.setPosition(turretSetpoint);
    }
    hood.setPosition(Math.max(0.0, Math.min(params.hoodPosition, 20.0)));
    if (CheetahUtil.isNear(turret.getPosition(), turretSetpoint, 10)
        && CheetahUtil.isNear(shooterSetpointRadPerSec, shooter.getMotorVelocity(), 20)) {
      indexer.VelocityVoltage(
          SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
          SmartDashboard.getNumber("indexerVerticleVelocity", 0));
      feeder.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
    } else {
      indexer.setMotorVoltage(0, 0);
      feeder.setFeederVoltage(0);
    }
    Logger.recordOutput("ShootOnMove/EffectiveTarget", params.effectiveTarget);
    Logger.recordOutput("ShootOnMove/TurretSetpointDeg", turretSetpointDeg);
    Logger.recordOutput("ShootOnMove/FinalDegree", turretSetpoint);
    Logger.recordOutput("ShootOnMove/ShooterSetpointRadPerSec", shooterSetpointRadPerSec);
    Logger.recordOutput("ShootOnMove/FieldTurretAngleRad", params.turretAngle.getRadians());
    Logger.recordOutput("ShootOnMove/HoodSetpoint", params.hoodPosition);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setMotorVoltage(0.0);
    indexer.setMotorVoltage(0, 0);
    feeder.setFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double wrapTo180(double degrees) {
    while (degrees > 180.0) {
      degrees -= 360.0;
    }
    while (degrees < -180.0) {
      degrees += 360.0;
    }
    return degrees;
  }
}
