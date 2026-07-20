package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HoodConstants;
import frc.robot.util.CheetahUtil;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Current> motorSupplyCurrent;

  private boolean hoodNeutral = false;

  public HoodIOTalonFX() {
    motor = new TalonFX(HoodConstants.kHoodMotorID, "canivore");

    motorConfigs.MotorOutput.withNeutralMode(
        HoodConstants.kMotorNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    motorConfigs.MotorOutput.withInverted(
        HoodConstants.kMotorInvertCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CheetahUtil.hoodDegToRotations(20);
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    motorConfigs.CurrentLimits.SupplyCurrentLimit = 20;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfigs.Slot0.kP = HoodConstants.kSlot_kP;
    motorConfigs.Slot0.kI = HoodConstants.kSlot_kI;
    motorConfigs.Slot0.kD = HoodConstants.kSlot_kD;
    motorConfigs.Slot0.kS = HoodConstants.kSlot_kS;
    motorConfigs.Slot0.kV = HoodConstants.kSlot_kV;

    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 10;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 40;

    motor.getConfigurator().apply(motorConfigs);

    motor.setPosition(0);

    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getTorqueCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        HoodConstants.kStatusUpdateFrequency,
        motorPosition,
        motorVelocity,
        motorAppliedVolts,
        motorCurrent,
        motorSupplyCurrent);

    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPosition, motorVelocity, motorAppliedVolts, motorCurrent, motorSupplyCurrent);

    inputs.motorPosition = motorPosition.getValueAsDouble();
    inputs.motorVelocityRotPerSec = motorVelocity.getValueAsDouble();
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();

    inputs.hoodPositionDeg = CheetahUtil.hoodRotationsToDeg(motorPosition.getValueAsDouble());
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double positionDeg) {
    // Talon expects rotations for position commands
    if (!hoodNeutral) {
      motor.setControl(m_request.withPosition(CheetahUtil.hoodDegToRotations(positionDeg)));
    } else {
      motor.setVoltage(0);
    }
  }

  @Override
  public void setHoodNeutral(boolean neutral) {
    hoodNeutral = neutral;
  }
}
