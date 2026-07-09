package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  // Hardware objects
  private final TalonFX flywheel;
  private TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
  private final TalonFX follower;
  private TalonFXConfiguration followerConfigs = new TalonFXConfiguration();
  private final TalonFX arm;
  private TalonFXConfiguration armConfigs = new TalonFXConfiguration();

  final MotionMagicVoltage m_armRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Angle> FlywheelPosition;
  private final StatusSignal<AngularVelocity> FlywheelVelocity;
  private final StatusSignal<Voltage> FlywheelAppliedVolts;
  private final StatusSignal<Current> FlywheelCurrent;

  private final StatusSignal<Angle> FollowerPosition;
  private final StatusSignal<AngularVelocity> FollowerVelocity;
  private final StatusSignal<Voltage> FollowerAppliedVolts;
  private final StatusSignal<Current> FollowerCurrent;

  private final StatusSignal<Angle> ArmPosition;
  private final StatusSignal<AngularVelocity> ArmVelocity;
  private final StatusSignal<Voltage> ArmAppliedVolts;
  private final StatusSignal<Current> ArmCurrent;

  public IntakeIOTalonFX() {
    flywheel = new TalonFX(IntakeConstants.flywheelID, "");
    follower = new TalonFX(IntakeConstants.followerID, "");
    flywheelConfigs.MotorOutput.withNeutralMode(
        IntakeConstants.flywheel_neutralmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerConfigs.MotorOutput.withNeutralMode(
        IntakeConstants.follower_neutralmode_Coast
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    flywheelConfigs.MotorOutput.withInverted(
        IntakeConstants.flywheel_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);
    followerConfigs.MotorOutput.withInverted(
        IntakeConstants.follower_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    arm = new TalonFX(IntakeConstants.armID, "");
    armConfigs.MotorOutput.withNeutralMode(
        IntakeConstants.arm_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    armConfigs.MotorOutput.withInverted(
        IntakeConstants.arm_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -12;

    // Set PID and feedforward constants from constants file
    armConfigs.Slot0.kP = IntakeConstants.armkP;
    armConfigs.Slot0.kI = IntakeConstants.armkI;
    armConfigs.Slot0.kD = IntakeConstants.armkD;
    armConfigs.Slot0.kA = IntakeConstants.armkA;
    armConfigs.Slot0.kS = IntakeConstants.armkS;
    armConfigs.Slot0.kV = IntakeConstants.armkV;

    armConfigs.MotionMagic.MotionMagicCruiseVelocity = 20;
    armConfigs.MotionMagic.MotionMagicAcceleration = 80;

    // Apply the configuration to the motor
    flywheel.getConfigurator().apply(flywheelConfigs);
    follower.getConfigurator().apply(followerConfigs);
    arm.getConfigurator().apply(armConfigs);

    // Create drive status signals
    FlywheelPosition = flywheel.getPosition();
    FlywheelVelocity = flywheel.getVelocity();
    FlywheelAppliedVolts = flywheel.getMotorVoltage();
    FlywheelCurrent = flywheel.getTorqueCurrent();

    FollowerPosition = follower.getPosition();
    FollowerVelocity = follower.getVelocity();
    FollowerAppliedVolts = follower.getMotorVoltage();
    FollowerCurrent = follower.getTorqueCurrent();

    ArmPosition = arm.getPosition();
    ArmVelocity = arm.getVelocity();
    ArmAppliedVolts = arm.getMotorVoltage();
    ArmCurrent = arm.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.statusrRegularUpdateFrequency,
        FlywheelAppliedVolts,
        FlywheelCurrent,
        FlywheelPosition,
        FlywheelVelocity,
        FollowerAppliedVolts,
        FollowerCurrent,
        FollowerPosition,
        FollowerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.statusrFasterUpdateFrequency,
        ArmPosition,
        ArmVelocity,
        ArmAppliedVolts,
        ArmCurrent);
    ParentDevice.optimizeBusUtilizationForAll(flywheel, follower, arm);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        FlywheelAppliedVolts,
        FlywheelCurrent,
        FlywheelPosition,
        FlywheelVelocity,
        FollowerAppliedVolts,
        FollowerCurrent,
        FollowerPosition,
        FollowerVelocity,
        ArmPosition,
        ArmVelocity,
        ArmAppliedVolts,
        ArmCurrent);
    // Update motor inputs
    inputs.FlywheelPositionRad = Units.rotationsToRadians(FlywheelPosition.getValueAsDouble());
    inputs.FlywheelVelocityRadPerSec =
        Units.rotationsToRadians(FlywheelVelocity.getValueAsDouble());
    inputs.FlywheelAppliedVolts = FlywheelAppliedVolts.getValueAsDouble();
    inputs.FlywheelCurrentAmps = FlywheelCurrent.getValueAsDouble();

    inputs.FollowerPositionRad = Units.rotationsToRadians(FollowerPosition.getValueAsDouble());
    inputs.FollowerVelocityRadPerSec =
        Units.rotationsToRadians(FollowerVelocity.getValueAsDouble());
    inputs.FollowerAppliedVolts = FollowerAppliedVolts.getValueAsDouble();
    inputs.FollowerCurrentAmps = FollowerCurrent.getValueAsDouble();

    inputs.ArmPositionRad = Units.rotationsToRadians(ArmPosition.getValueAsDouble());
    inputs.ArmVelocityRadPerSec = Units.rotationsToRadians(ArmVelocity.getValueAsDouble());
    inputs.ArmAppliedVolts = ArmAppliedVolts.getValueAsDouble();
    inputs.ArmCurrentAmps = ArmCurrent.getValueAsDouble();
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheel.setVoltage(volts);
    follower.setControl(new Follower(IntakeConstants.flywheelID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setArmVoltage(double volts) {
    arm.setVoltage(volts);
  }

  @Override
  public void setArmPosition(double radians) {
    arm.setControl(m_armRequest.withPosition(Units.radiansToRotations(radians)));
  }
}
