package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import org.littletonrobotics.junction.Logger;

/**
 * Aims the turret and spins the flywheel to shoot at the alliance target while the robot is moving.
 *
 * <p>The driver retains full control of chassis translation and rotation. This command reads the
 * robot's field-relative pose and chassis speeds, runs the shoot-on-the-move solver, and commands
 * the turret and shooter.
 */
public class ShootOnMoveDefaultCommand extends Command {
  private final Drive drive;
  private final TurretSubsystem turret;
  private final ShootOnMoveCalculator calculator;

  public ShootOnMoveDefaultCommand(
      Drive drive, TurretSubsystem turret, ShootOnMoveCalculator calculator) {
    this.drive = drive;
    this.turret = turret;
    this.calculator = calculator;

    addRequirements(turret);
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

    turret.setPosition(wrapTo180(turretSetpointDeg + turretOffset));

    Logger.recordOutput("ShootOnMove/EffectiveTarget", params.effectiveTarget);
    Logger.recordOutput("ShootOnMove/TurretSetpointDeg", turretSetpointDeg);
    Logger.recordOutput("ShootOnMove/FinalDegree", wrapTo180(turretSetpointDeg + turretOffset));
    Logger.recordOutput("ShootOnMove/ShooterSetpointRadPerSec", shooterSetpointRadPerSec);
    Logger.recordOutput("ShootOnMove/FieldTurretAngleRad", params.turretAngle.getRadians());
  }

  @Override
  public void end(boolean interrupted) {}

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
