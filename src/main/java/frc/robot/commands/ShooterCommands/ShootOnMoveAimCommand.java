package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;
import org.littletonrobotics.junction.Logger;

public abstract class ShootOnMoveAimCommand extends Command {
  protected final Drive drive;
  protected final TurretSubsystem turret;
  protected final ShootOnMoveCalculator calculator;

  protected record AimResult(double turretSetpointDeg, ShotParameters parameters) {}

  protected ShootOnMoveAimCommand(
      Drive drive, TurretSubsystem turret, ShootOnMoveCalculator calculator) {
    this.drive = drive;
    this.turret = turret;
    this.calculator = calculator;
    addRequirements(turret);
  }

  @Override
  public final void execute() {
    AimResult result = computeAim();
    turret.setPosition(result.turretSetpointDeg());
    onAim(result);
  }

  protected final AimResult computeAim() {
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, drive.getRotation());
    ShotParameters params = calculator.calculate(drive.getPose(), fieldRelativeSpeeds);

    double turretSetpointDeg = params.turretAngle.minus(drive.getRotation()).getDegrees();
    double turretOffset = 60;
    double turretSetpoint = wrapTo180(turretSetpointDeg + turretOffset);
    if (turretSetpoint > 170) {
      turretSetpoint = 170;
    } else if (turretSetpoint < -170) {
      turretSetpoint = -170;
    }

    Logger.recordOutput("ShootOnMove/EffectiveTarget", params.effectiveTarget);
    Logger.recordOutput("ShootOnMove/TurretSetpointDeg", turretSetpointDeg);
    Logger.recordOutput("ShootOnMove/FinalDegree", turretSetpoint);
    Logger.recordOutput(
        "ShootOnMove/ShooterSetpointRadPerSec", params.flywheelSpeedRps * 2.0 * Math.PI);
    Logger.recordOutput(
        "ShootOnMove/FarShooterSetpointRadPerSec", params.farFlyWheelSpeedRps * 2.0 * Math.PI);
    Logger.recordOutput("ShootOnMove/FieldTurretAngleRad", params.turretAngle.getRadians());

    return new AimResult(turretSetpoint, params);
  }

  protected abstract void onAim(AimResult result);

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
