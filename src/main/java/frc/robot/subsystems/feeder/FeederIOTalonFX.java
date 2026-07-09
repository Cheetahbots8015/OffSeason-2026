package frc.robot.subsystems.feeder;

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
import frc.robot.constants.FeederConstants;

public class FeederIOTalonFX implements FeederIO {
  // Hardware objects
  private final TalonFX feeder;

  private TalonFXConfiguration feederConfigs = new TalonFXConfiguration();

  // Voltage control requests
  final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(0);

  // Inputs from motor
  private final StatusSignal<Angle> FeederPosition;
  private final StatusSignal<AngularVelocity> FeederVelocity;
  private final StatusSignal<Voltage> FeederAppliedVolts;
  private final StatusSignal<Current> FeederCurrent;

  public FeederIOTalonFX() {
    feeder = new TalonFX(FeederConstants.feederID, "canivore");

    feederConfigs.MotorOutput.withNeutralMode(
        FeederConstants.feeder_neutralmode_Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    // Set motor inversion based on desired rotation direction
    feederConfigs.MotorOutput.withInverted(
        FeederConstants.feeder_inverted_CounterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    // Set PID and feedforward constants from constants file
    feederConfigs.Slot0.kP = FeederConstants.feederkP;
    feederConfigs.Slot0.kI = FeederConstants.feederkI;
    feederConfigs.Slot0.kD = FeederConstants.feederkD;
    feederConfigs.Slot0.kV = FeederConstants.feederkV;

    // Apply the configuration to the motor
    feeder.getConfigurator().apply(feederConfigs);

    // Create drive status signals
    FeederPosition = feeder.getPosition();
    FeederVelocity = feeder.getVelocity();
    FeederAppliedVolts = feeder.getMotorVoltage();
    FeederCurrent = feeder.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        FeederConstants.statusUpdateFrequency,
        FeederPosition,
        FeederVelocity,
        FeederAppliedVolts,
        FeederCurrent);
    ParentDevice.optimizeBusUtilizationForAll(feeder);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(FeederPosition, FeederVelocity, FeederAppliedVolts, FeederCurrent);
    // Update motor inputs
    inputs.FeederPositionRad = Units.rotationsToRadians(FeederPosition.getValueAsDouble());
    inputs.FeederVelocityRotPerSec = FeederVelocity.getValueAsDouble();
    inputs.FeederAppliedVolts = FeederAppliedVolts.getValueAsDouble();
    inputs.FeederCurrentAmps = FeederCurrent.getValueAsDouble();
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setVoltage(volts);
  }

  @Override
  public void setFeederVelocityVoltage(double velocity) {
    feeder.setControl(m_velocity.withVelocity(velocity));
  }
}
