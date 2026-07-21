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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.ContainerConstants;
import frc.robot.constants.ContainerConstants.Mode;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends FullSubsystem {
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.robotMassKg,
          DriveConstants.robotMOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              DriveConstants.wheelCOF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  // Offset between the Pigeon's yaw and the robot's true heading. A Pigeon reboot
  // re-zeros yaw; the offset re-anchors it to the heading we coasted to via module twists.
  private Rotation2d gyroOffset = new Rotation2d();
  private boolean wasGyroConnected = true;
  private int gyroRebootCount = 0;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private ChassisSpeeds preSpeeds;
  private RobotConfig robotconfig;

  public boolean autoFliped = false;

  private Field2d field2d;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.1), new PIDConstants(2.5, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    try {
      robotconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> modules[0].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> modules[0].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> modules[1].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> modules[1].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> modules[2].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> modules[2].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> modules[3].getAngle().getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> modules[3].getVelocityMetersPerSec(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
          }
        });
    field2d = new Field2d();
  }

  /**
   * Returns a short rejection reason for a MegaTag2 frame, or null if the frame should be fused.
   * Logged per camera so field debugging can distinguish "camera blind" from "code rejecting".
   */
  private String visionRejectionReason(PoseEstimate mt2, int[] validateID) {
    if (mt2.tagCount == 0) {
      return "NoTags";
    }
    // While the gyro is down the heading seed is twist-integrated, not measured —
    // only accept multi-tag frames, whose heading is constrained by the tags themselves.
    if (!gyroInputs.connected && mt2.tagCount == 1) {
      return "GyroDisconnected";
    }
    // MT2's yaw seed lags one frame; frames captured while spinning fast are the worst
    // (and are exactly the post-collision frames). Odometry is reliable here, so reject.
    if (Math.abs(gyroInputs.yawVelocityRadPerSec) > VisionConstants.maxGyroRateRadPerSec) {
      return "GyroRate";
    }
    if (mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
      if (mt2.rawFiducials[0].ambiguity > VisionConstants.maxAmbiguity) {
        return "Ambiguity";
      }
      if (mt2.rawFiducials[0].distToCamera > VisionConstants.maxSingleTagDistMeters) {
        return "Distance";
      }
      boolean allowed = false;
      for (int id : validateID) {
        if (mt2.rawFiducials[0].id == id) {
          allowed = true;
          break;
        }
      }
      if (!allowed) {
        return "InvalidID";
      }
    } else if (mt2.avgTagDist > VisionConstants.maxMultiTagDistMeters) {
      return "Distance";
    }
    return null;
  }

  /**
   * Computes the XY stddev (meters) for an accepted MT2 frame — how much the pose estimator trusts
   * this measurement. Lower = trusted more = pose snaps toward vision faster.
   *
   * <p>Model: scale with the square of average tag distance (uncertainty grows fast with range) and
   * shrink with the square of tag count (multi-tag fixes are much better). Apply the per-camera
   * factor, then clamp to a small floor so even perfect fixes don't get infinite trust. Theta is
   * never taken from vision (see VisionConstants.thetaStdDev).
   *
   * <p>Reference starting point from the investigation doc (MT2 needs a bit more trust than 6328's
   * 0.01 coefficient, which was tuned for their own solver):
   *
   * <pre>
   * stddev = max(floor, baseline * avgTagDist^2 / tagCount^2 * cameraStdDevFactor)
   * </pre>
   *
   * <p>Trade-off to tune on the field: a lower baseline/floor re-converges faster after a collision
   * but jitters more when tags are marginal.
   */
  private double calculateVisionStdDev(PoseEstimate mt2, CameraConfig camera) {
    double distSq = mt2.avgTagDist * mt2.avgTagDist;
    double tagSq = mt2.tagCount * mt2.tagCount;
    double stdDev = VisionConstants.xyStdDevBaseline * distSq / tagSq * camera.stdDevFactor();
    return Math.max(VisionConstants.xyStdDevFloor, stdDev);
  }

  private void processCamera(CameraConfig camera, int[] validateID) {
    String logKey = "LL/" + camera.name().replace("limelight-", "");
    String rejection;
    try {
      // Seed MT2 with the offset-corrected gyro heading + raw yaw rate. rawGyroRotation
      // stays valid through gyro reboots (offset re-anchoring + twist coasting), unlike
      // the raw gyro yaw which re-zeros. MT2 uses the rate to interpolate yaw at frame
      // capture time.
      LimelightHelpers.SetRobotOrientation(
          camera.name(),
          rawGyroRotation.getDegrees(),
          Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec),
          0,
          0,
          0,
          0);
      LimelightHelpers.SetFiducialIDFiltersOverride(camera.name(), validateID);
      PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera.name());

      if (mt2 == null) {
        rejection = "NoEstimate";
      } else {
        rejection = visionRejectionReason(mt2, validateID);
        if (rejection == null) {
          double xyStdDev = calculateVisionStdDev(mt2, camera);
          poseEstimator.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds,
              VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.thetaStdDev));
          Logger.recordOutput(logKey + "/stddev", xyStdDev);
        }
        Logger.recordOutput(logKey + "/pose", mt2.pose);
        Logger.recordOutput(logKey + "/timestamp", mt2.timestampSeconds);
        Logger.recordOutput(logKey + "/avgdist", mt2.avgTagDist);
        Logger.recordOutput(logKey + "/latency", mt2.latency);
        Logger.recordOutput(logKey + "/tagCount", mt2.tagCount);
      }
    } catch (Exception e) {
      rejection = "Exception";
      Logger.recordOutput(logKey + "/error", e.toString());
    }
    Logger.recordOutput(logKey + "/rejection", rejection == null ? "None" : rejection);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Drive", this);
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock(); // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Gyro reconnect (e.g. after a power cycle): the Pigeon re-zeros yaw on boot,
    // so re-anchor the offset to the heading we coasted to via module twists.
    if (gyroInputs.connected && !wasGyroConnected) {
      gyroOffset = rawGyroRotation.minus(gyroInputs.yawPosition);
      gyroRebootCount++;
      Logger.recordOutput("Drive/Gyro/RebootOffsetDeg", gyroOffset.getDegrees());
    }
    wasGyroConnected = gyroInputs.connected;
    Logger.recordOutput("Drive/Gyro/RebootCount", gyroRebootCount);

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
        // Use the real gyro angle, corrected by the reboot offset
        Rotation2d adjustedYaw = gyroInputs.odometryYawPositions[i].plus(gyroOffset);
        // A yaw jump too large for one odometry sample means the gyro re-zeroed
        // without a detected disconnect — re-anchor the offset on the fly.
        if (Math.abs(adjustedYaw.minus(rawGyroRotation).getRadians())
            > DriveConstants.gyroRezeroJumpThresholdRad) {
          gyroOffset = rawGyroRotation.minus(gyroInputs.odometryYawPositions[i]);
          adjustedYaw = rawGyroRotation;
          gyroRebootCount++;
          Logger.recordOutput("Drive/Gyro/RebootOffsetDeg", gyroOffset.getDegrees());
        }
        rawGyroRotation = adjustedYaw;
      } else {
        // Coast on module twists (gyro disconnected or sample missing)
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    int[] validateID = DriveConstants.blueTags;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      validateID = DriveConstants.redTags;
    }
    for (var camera : VisionConstants.cameras) {
      processCamera(camera, validateID);
    }

    SmartDashboard.putNumberArray(
        "translation",
        new double[] {
          poseEstimator.getEstimatedPosition().getTranslation().getX(),
          poseEstimator.getEstimatedPosition().getTranslation().getY()
        });
    SmartDashboard.putNumber(
        "rotation", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    // Update gyro alert
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field2d", field2d);
    gyroDisconnectedAlert.set(!gyroInputs.connected && ContainerConstants.currentMode != Mode.SIM);
  }

  @Override
  public void periodicAfterScheduler() {
    // Apply the dynamic drive current limit from the energy budget
    for (var module : modules) {
      module.applyEnergyLimit();
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
