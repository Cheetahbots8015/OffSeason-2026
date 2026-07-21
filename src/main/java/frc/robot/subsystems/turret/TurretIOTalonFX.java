package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.TurretConstants;
import frc.robot.util.CheetahUtil;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final Pigeon2 pigeon;
  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0).withSlot(0);

  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Angle> pigeonYaw;
  private boolean turretLocked;

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.kTurretMotorID, "");
    pigeon = new Pigeon2(TurretConstants.kPigeonId, "canivore");

    motorConfigs.MotorOutput.withNeutralMode(
        TurretConstants.kMotorNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    motorConfigs.MotorOutput.withInverted(
        TurretConstants.kMotorInvertCCWPositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        CheetahUtil.turretDegToRotations(-175);
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        CheetahUtil.turretDegToRotations(175);
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    motorConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfigs.Slot0.kP = TurretConstants.kSlot_kP;
    motorConfigs.Slot0.kI = TurretConstants.kSlot_kI;
    motorConfigs.Slot0.kD = TurretConstants.kSlot_kD;
    motorConfigs.Slot0.kS = TurretConstants.kSlot_kS;
    motorConfigs.Slot0.kV = TurretConstants.kSlot_kV;

    motorConfigs.Voltage.PeakForwardVoltage = 3.0;
    motorConfigs.Voltage.PeakReverseVoltage = -3.0;

    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 10;
    motorConfigs.MotionMagic.MotionMagicAcceleration = 40;

    pigeon.setYaw(0);

    motor.getConfigurator().apply(motorConfigs);
    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getTorqueCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();
    pigeonYaw = pigeon.getYaw();
    turretLocked = false;

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.kStatusUpdateFrequency,
        motorPosition,
        motorVelocity,
        motorAppliedVolts,
        motorCurrent,
        motorSupplyCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(TurretConstants.kStatusUpdateFrequency, pigeonYaw);

    ParentDevice.optimizeBusUtilizationForAll(motor, pigeon);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorPosition, motorVelocity, motorAppliedVolts, motorCurrent, motorSupplyCurrent);
    BaseStatusSignal.refreshAll(pigeonYaw);

    inputs.motorPositionDeg = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
    inputs.motorVelocityRotPerSec = motorVelocity.getValueAsDouble();
    inputs.motorAppliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrentAmps = motorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();

    inputs.turretPositionDeg = CheetahUtil.turretRotationsToDeg(motorPosition.getValueAsDouble());

    inputs.pigeonYawDeg = pigeonYaw.getValueAsDouble();
    inputs.calculatedRobotDeg = inputs.pigeonYawDeg - inputs.turretPositionDeg;

    inputs.turretLocked = turretLocked;
  }

  @Override
  public void setMotorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setPosition(double positionDeg) {
    // Talon expects rotations for position commands\
    if (!turretLocked) {
      motor.setControl(m_request.withPosition(CheetahUtil.turretDegToRotations(positionDeg)));
    } else {
      motor.setControl(m_request.withPosition(CheetahUtil.turretDegToRotations(60)));
    }
  }

  @Override
  public void lockTurret(boolean lock) {
    if (lock) {
      turretLocked = true;
    } else {
      turretLocked = false;
    }
  }
}
