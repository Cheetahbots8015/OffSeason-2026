package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootPassCommand extends Command {
  private final Drive drive;
  private final TurretSubsystem turret;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final FeederSubsystem feeder;

  protected record AimResult(double turretSetpointDeg) {}

  public ShootPassCommand(
      Drive drive,
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.indexer = indexer;
    this.feeder = feeder;
    addRequirements(turret, shooter, indexer, feeder);
  }

  @Override
  public final void execute() {
    AimResult result = computeAim();
    turret.setPosition(result.turretSetpointDeg());
    shooter.VelocityVoltage(300);
    indexer.VelocityVoltage(
        SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
        SmartDashboard.getNumber("indexerVerticleVelocity", 0));
    feeder.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
  }

  protected final AimResult computeAim() {

    double turretSetpointDeg =
        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
            ? new Rotation2d(0).minus(drive.getRotation()).getDegrees()
            : new Rotation2d(Math.PI).minus(drive.getRotation()).getDegrees();
    double turretOffset = 60;
    double turretSetpoint = wrapTo180(turretSetpointDeg + turretOffset);
    if (turretSetpoint > 170) {
      turretSetpoint = 170;
    } else if (turretSetpoint < -170) {
      turretSetpoint = -170;
    }

    return new AimResult(turretSetpoint);
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
}
