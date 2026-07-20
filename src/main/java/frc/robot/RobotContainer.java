// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoodCommands.HoodNeutral;
import frc.robot.commands.IntakeCommands.IntakeArmCommand;
import frc.robot.commands.IntakeCommands.IntakeAutoCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeDriveCommand;
import frc.robot.commands.IntakeCommands.IntakeSHMCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ShooterCommands.ShootFarOnMoveTriggerCommand;
import frc.robot.commands.ShooterCommands.ShootOnMoveDefaultCommand;
import frc.robot.commands.ShooterCommands.ShootOnMoveTriggerCommand;
import frc.robot.commands.TurretCommands.TurretLock;
import frc.robot.commands.TurretCommands.TurretPositionVoltage;
import frc.robot.constants.ContainerConstants;
import frc.robot.constants.ShootOnMoveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private final CommandXboxController controller = new CommandXboxController(0);

  private final CommandXboxController subController = new CommandXboxController(1);

  private final Drive drive;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final ShootOnMoveCalculator shootOnMoveCalculator;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (ContainerConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2() {},
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        feeder = new FeederSubsystem(new FeederIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        indexer = new IndexerSubsystem(new IndexerOTalonFX());
        turret = new TurretSubsystem(new TurretIOTalonFX());
        hood = new HoodSubsystem(new HoodIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new IntakeSubsystem(new IntakeIOSim());
        feeder = new FeederSubsystem(new FeederIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        indexer = new IndexerSubsystem(new IndexerIOSim());
        turret = new TurretSubsystem(new TurretIOSim());
        hood = new HoodSubsystem(new HoodIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new IntakeSubsystem(new IntakeIOSim());
        feeder = new FeederSubsystem(new FeederIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSim());
        indexer = new IndexerSubsystem(new IndexerIOSim());
        turret = new TurretSubsystem(new TurretIOSim());
        hood = new HoodSubsystem(new HoodIOSim());
        break;
    }

    shootOnMoveCalculator =
        new ShootOnMoveCalculator(
            ShootOnMoveConstants.kRedTarget, ShootOnMoveConstants.kBlueTarget);

    // Set up auto routines
    NamedCommands.registerCommand("Intake4", new IntakeAutoCommand(intake, 4));
    NamedCommands.registerCommand(
        "Shoot",
        new ShootOnMoveTriggerCommand(
            drive, turret, hood, shooter, indexer, feeder, shootOnMoveCalculator));
    NamedCommands.registerCommand(
        "LongShoot",
        new ShootFarOnMoveTriggerCommand(
            drive, turret, hood, shooter, indexer, feeder, shootOnMoveCalculator));
    NamedCommands.registerCommand("IntakeSHM", new IntakeSHMCommand(intake));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    /*
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */

    SmartDashboard.putNumber("feederVolt", 5);
    SmartDashboard.putNumber("shooterVolt", 5);
    SmartDashboard.putNumber("shooterVelocity", 250);
    SmartDashboard.putNumber("indexerHorizontalVolt", 4);
    SmartDashboard.putNumber("indexerVerticleVolt", 4);
    SmartDashboard.putNumber("indexerHorizontalVelocity", 250);
    SmartDashboard.putNumber("indexerVerticleVelocity", 250);
    SmartDashboard.putNumber("feederVelocity", 300);

    SmartDashboard.putNumber("shooterOffset", 0);

    // Set up SysId routines
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -controller.getLeftY() > 0
                    ? Math.pow(controller.getLeftY(), 2)
                    : -Math.pow(controller.getLeftY(), 2),
            () ->
                -controller.getLeftX() > 0
                    ? Math.pow(controller.getLeftX(), 2)
                    : -Math.pow(controller.getLeftX(), 2),
            () -> -controller.getRightX()));
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Rotation2d heading =
                          DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
                              ? new Rotation2d(Math.PI)
                              : new Rotation2d();

                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), heading));
                    },
                    drive)
                .ignoringDisable(true));

    controller.povDown().whileTrue(new TurretPositionVoltage(turret, 0));
    controller.leftTrigger().whileTrue(new IntakeCommand(intake));
    turret.setDefaultCommand(
        new ShootOnMoveDefaultCommand(drive, turret, hood, shootOnMoveCalculator));

    controller
        .rightTrigger()
        .and(subController.a().negate())
        .whileTrue(
            new ShootOnMoveTriggerCommand(
                drive, turret, hood, shooter, indexer, feeder, shootOnMoveCalculator));
    controller
        .rightTrigger()
        .and(subController.a())
        .whileTrue(
            new ShootFarOnMoveTriggerCommand(
                drive, turret, hood, shooter, indexer, feeder, shootOnMoveCalculator));

    subController.povLeft().onTrue(new TurretLock(turret, true));
    subController.povRight().onTrue(new TurretLock(turret, false));
    subController.x().onTrue(new HoodNeutral(hood, true));
    subController.b().onTrue(new HoodNeutral(hood, false));
    subController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () ->
                    SmartDashboard.putNumber(
                        "shooterOffset", SmartDashboard.getNumber("shooterOffset", 0.0) + 2.5)));
    subController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () ->
                    SmartDashboard.putNumber(
                        "shooterOffset", SmartDashboard.getNumber("shooterOffset", 0.0) - 2.5)));

    /*
    controller.rightTrigger().whileTrue(new ShootVelocityVoltageCommand(shooter));
    controller.rightTrigger().whileTrue(new IndexerVelocityVoltageCommand(indexer, controller));
    controller.rightTrigger().whileTrue(new FeederVelocityVoltageCommand(feeder));
    */

    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () ->
                    -controller.getLeftY() > 0
                        ? Math.pow(controller.getLeftY(), 2) * 0.5
                        : -Math.pow(controller.getLeftY(), 2) * 0.5,
                () ->
                    -controller.getLeftX() > 0
                        ? Math.pow(controller.getLeftX(), 2) * 0.5
                        : -Math.pow(controller.getLeftX(), 2) * 0.5,
                () -> -controller.getRightX() * 0.2));
    controller.rightTrigger().whileTrue(new IntakeSHMCommand(intake));
    controller.x().whileTrue(new OutakeCommand(indexer, feeder));
    controller.rightBumper().whileTrue(new IntakeDriveCommand(intake, 2));
    controller.leftBumper().whileTrue(new IntakeArmCommand(intake, -75));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
