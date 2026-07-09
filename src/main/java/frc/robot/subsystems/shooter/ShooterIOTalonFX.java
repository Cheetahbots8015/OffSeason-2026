package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left;
  private final TalonFX right;

  private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  // Status signals for telemetry and odometry
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftCurrent;

  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightCurrent;

  public ShooterIOTalonFX() {
    left = new TalonFX(ShooterConstants.kLeftMotorID, "canivore");
    right = new TalonFX(ShooterConstants.kRightMotorID, "canivore");

    TalonFXConfiguration leftConfigs = new TalonFXConfiguration();
    TalonFXConfiguration rightConfigs = new TalonFXConfiguration();

    // Configure neutral mode
    leftConfigs.MotorOutput.NeutralMode =
        ShooterConstants.kLeftNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    rightConfigs.MotorOutput.NeutralMode =
        ShooterConstants.kRightNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    leftConfigs.MotorOutput.Inverted =
        ShooterConstants.kLeftInvert
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    rightConfigs.MotorOutput.Inverted =
        ShooterConstants.kRightInvert
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    leftConfigs.Slot0.kP = ShooterConstants.kP;
    leftConfigs.Slot0.kI = ShooterConstants.kI;
    leftConfigs.Slot0.kD = ShooterConstants.kD;
    leftConfigs.Slot0.kV = ShooterConstants.kV;
    rightConfigs.Slot0.kP = ShooterConstants.kP;
    rightConfigs.Slot0.kI = ShooterConstants.kI;
    rightConfigs.Slot0.kD = ShooterConstants.kD;
    rightConfigs.Slot0.kV = ShooterConstants.kV;

    left.getConfigurator().apply(leftConfigs);
    right.getConfigurator().apply(rightConfigs);

    leftPosition = left.getPosition();
    leftVelocity = left.getVelocity();
    leftAppliedVolts = left.getMotorVoltage();
    leftCurrent = left.getTorqueCurrent();
    rightPosition = right.getPosition();
    rightVelocity = right.getVelocity();
    rightAppliedVolts = right.getMotorVoltage();
    rightCurrent = right.getTorqueCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.kStatusUpdateFrequency,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);

    ParentDevice.optimizeBusUtilizationForAll(left, right);
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightCurrent);

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double volts) {
    right.setVoltage(volts);
    left.setControl(new Follower(ShooterConstants.kRightMotorID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void VelocityVoltage(double radians) {
    right.setControl(m_velocityVoltage.withVelocity(Units.radiansToRotations(radians)));
    left.setControl(new Follower(ShooterConstants.kRightMotorID, MotorAlignmentValue.Opposed));
  }
}
