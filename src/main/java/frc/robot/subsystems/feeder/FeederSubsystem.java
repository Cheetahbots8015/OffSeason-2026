// ExampleSubsystem - Subsystem to control a single TalonFX motor for a example

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void stop() {
    io.setFeederVoltage(0);
  }

  public FeederIO getIO() {
    return io;
  }

  public FeederIOInputs getInput() {
    return inputs;
  }

  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(volts);
  }

  public void setFeederVelocityVoltage(double velocity) {
    io.setFeederVelocityVoltage(velocity);
  }
}
