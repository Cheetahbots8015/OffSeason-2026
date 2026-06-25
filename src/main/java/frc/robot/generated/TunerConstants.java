package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class TunerConstants {
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(2.05)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs[] driveGains = {
    new Slot0Configs()
        .withKP(2)
        .withKI(0)
        .withKD(0)
        .withKS(0.30373)
        .withKV(0.73263)
        .withKA(0.046318),
    new Slot0Configs()
        .withKP(2)
        .withKI(0)
        .withKD(0)
        .withKS(0.28681)
        .withKV(0.7175)
        .withKA(0.045239),
    new Slot0Configs()
        .withKP(2)
        .withKI(0)
        .withKD(0)
        .withKS(0.25308)
        .withKV(0.75563)
        .withKA(0.11529),
    new Slot0Configs()
        .withKP(2)
        .withKI(0)
        .withKD(0)
        .withKS(0.15459)
        .withKV(0.72295)
        .withKA(0.085011)
  };

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  private static final Current kSlipCurrent = Amps.of(120.0);

  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimit(Amps.of(30))
                  .withSupplyCurrentLimitEnable(true))
          .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
  private static final TalonFXConfiguration driveHighLoadConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(false)
                  .withSupplyCurrentLimit(Amps.of(40))
                  .withSupplyCurrentLimitEnable(true))
          .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));

  private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.212);
  private static final double kCoupleRatio = 3.5714285714285716;

  private static final double kDriveGearRatio = 6.12;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final Distance kWheelRadius = Inches.of(2.079);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final int kPigeonId = 5;

  // These are only used for simulation
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
  // Simulated voltage necessary to overcome friction
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 31;
  private static final int kFrontLeftSteerMotorId = 32;
  private static final int kFrontLeftEncoderId = 3;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.37329);
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(10.425);
  private static final Distance kFrontLeftYPos = Inches.of(10.425);

  // Front Right
  private static final int kFrontRightDriveMotorId = 21;
  private static final int kFrontRightSteerMotorId = 22;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.29785);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10.425);
  private static final Distance kFrontRightYPos = Inches.of(-10.425);

  // Back Left
  private static final int kBackLeftDriveMotorId = 41;
  private static final int kBackLeftSteerMotorId = 42;
  private static final int kBackLeftEncoderId = 4;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.246582);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-10.425);
  private static final Distance kBackLeftYPos = Inches.of(10.425);

  // Back Right
  private static final int kBackRightDriveMotorId = 11;
  private static final int kBackRightSteerMotorId = 12;
  private static final int kBackRightEncoderId = 1;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.723);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-10.425);
  private static final Distance kBackRightYPos = Inches.of(-10.425);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
                  kFrontLeftSteerMotorId,
                  kFrontLeftDriveMotorId,
                  kFrontLeftEncoderId,
                  kFrontLeftEncoderOffset,
                  kFrontLeftXPos,
                  kFrontLeftYPos,
                  kInvertLeftSide,
                  kFrontLeftSteerMotorInverted,
                  kFrontLeftEncoderInverted)
              .withDriveMotorGains(driveGains[0])
              .withDriveMotorInitialConfigs(driveInitialConfigs);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
                  kFrontRightSteerMotorId,
                  kFrontRightDriveMotorId,
                  kFrontRightEncoderId,
                  kFrontRightEncoderOffset,
                  kFrontRightXPos,
                  kFrontRightYPos,
                  kInvertRightSide,
                  kFrontRightSteerMotorInverted,
                  kFrontRightEncoderInverted)
              .withDriveMotorGains(driveGains[1])
              .withDriveMotorInitialConfigs(driveInitialConfigs);
  ;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
                  kBackLeftSteerMotorId,
                  kBackLeftDriveMotorId,
                  kBackLeftEncoderId,
                  kBackLeftEncoderOffset,
                  kBackLeftXPos,
                  kBackLeftYPos,
                  kInvertLeftSide,
                  kBackLeftSteerMotorInverted,
                  kBackLeftEncoderInverted)
              .withDriveMotorGains(driveGains[2])
              .withDriveMotorInitialConfigs(driveHighLoadConfigs);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
                  kBackRightSteerMotorId,
                  kBackRightDriveMotorId,
                  kBackRightEncoderId,
                  kBackRightEncoderOffset,
                  kBackRightXPos,
                  kBackRightYPos,
                  kInvertRightSide,
                  kBackRightSteerMotorInverted,
                  kBackRightEncoderInverted)
              .withDriveMotorGains(driveGains[3])
              .withDriveMotorInitialConfigs(driveHighLoadConfigs);

  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
