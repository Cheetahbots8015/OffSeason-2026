package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IndexerConstants;

public class IndexerOTalonFX implements IndexerIO {
  private final TalonFX horizontal;
  private final TalonFX verticle;

  private VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
  // Status signals for telemetry and odometry
  private final StatusSignal<Angle> horizontalPosition;
  private final StatusSignal<AngularVelocity> horizontalVelocity;
  private final StatusSignal<Voltage> horizontalAppliedVolts;
  private final StatusSignal<Current> horizontalCurrent;

  private final StatusSignal<Angle> verticlePosition;
  private final StatusSignal<AngularVelocity> verticleVelocity;
  private final StatusSignal<Voltage> verticleAppliedVolts;
  private final StatusSignal<Current> verticleCurrent;

  public IndexerOTalonFX() {
    horizontal = new TalonFX(IndexerConstants.kHoriMotorID, "");
    verticle = new TalonFX(IndexerConstants.kVertMotorID, "");

    TalonFXConfiguration horizontalConfigs = new TalonFXConfiguration();
    TalonFXConfiguration verticleConfigs = new TalonFXConfiguration();

    // Configure neutral mode
    horizontalConfigs.MotorOutput.NeutralMode =
        IndexerConstants.kHoriNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    verticleConfigs.MotorOutput.NeutralMode =
        IndexerConstants.kVertNeutralCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    horizontalConfigs.MotorOutput.Inverted =
        IndexerConstants.kHoriInvert
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    verticleConfigs.MotorOutput.Inverted =
        IndexerConstants.kVertInvert
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    horizontalConfigs.Slot0.kP = IndexerConstants.kP;
    horizontalConfigs.Slot0.kI = IndexerConstants.kI;
    horizontalConfigs.Slot0.kD = IndexerConstants.kD;
    horizontalConfigs.Slot0.kV = IndexerConstants.kV;
    verticleConfigs.Slot0.kP = IndexerConstants.kP;
    verticleConfigs.Slot0.kI = IndexerConstants.kI;
    verticleConfigs.Slot0.kD = IndexerConstants.kD;
    verticleConfigs.Slot0.kV = IndexerConstants.kV;

    horizontal.getConfigurator().apply(horizontalConfigs);
    verticle.getConfigurator().apply(verticleConfigs);

    horizontalPosition = horizontal.getPosition();
    horizontalVelocity = horizontal.getVelocity();
    horizontalAppliedVolts = horizontal.getMotorVoltage();
    horizontalCurrent = horizontal.getTorqueCurrent();
    verticlePosition = verticle.getPosition();
    verticleVelocity = verticle.getVelocity();
    verticleAppliedVolts = verticle.getMotorVoltage();
    verticleCurrent = verticle.getTorqueCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        IndexerConstants.kStatusUpdateFrequency,
        horizontalPosition,
        horizontalVelocity,
        horizontalAppliedVolts,
        horizontalCurrent,
        verticlePosition,
        verticleVelocity,
        verticleAppliedVolts,
        verticleCurrent);

    ParentDevice.optimizeBusUtilizationForAll(horizontal, verticle);
  }

  @Override
  public void updateInputs(IndexerIO.IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        horizontalPosition,
        horizontalVelocity,
        horizontalAppliedVolts,
        horizontalCurrent,
        verticlePosition,
        verticleVelocity,
        verticleAppliedVolts,
        verticleCurrent);

    inputs.horizontalPositionRad = Units.rotationsToRadians(horizontalPosition.getValueAsDouble());
    inputs.horizontalVelocityRadPerSec =
        Units.rotationsToRadians(horizontalVelocity.getValueAsDouble());
    inputs.horizontalAppliedVolts = horizontalAppliedVolts.getValueAsDouble();
    inputs.horizontalCurrentAmps = horizontalCurrent.getValueAsDouble();
    inputs.verticlePositionRad = Units.rotationsToRadians(verticlePosition.getValueAsDouble());
    inputs.verticleVelocityRadPerSec =
        Units.rotationsToRadians(verticleVelocity.getValueAsDouble());
    inputs.verticleAppliedVolts = verticleAppliedVolts.getValueAsDouble();
    inputs.verticleCurrentAmps = verticleCurrent.getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double horivolts, double vertvolts) {
    verticle.setVoltage(vertvolts);
    horizontal.setVoltage(horivolts);
  }

  @Override
  public void VelocityVoltage(double horiradians, double vertradians) {
    verticle.setControl(m_VelocityVoltage.withVelocity(Units.radiansToRotations(vertradians)));
    horizontal.setControl(m_VelocityVoltage.withVelocity(Units.radiansToRotations(horiradians)));
  }
}
