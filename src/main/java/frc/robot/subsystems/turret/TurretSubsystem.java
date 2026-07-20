package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.FinanceDepartment;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  public TurretSubsystem(TurretIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setMotorVoltage(voltage.in(Units.Volt)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Report energy usage (supply current = battery draw)
    FinanceDepartment.getInstance()
        .reportCurrentUsage("Turret", false, inputs.motorSupplyCurrentAmps);
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /** Set turret position in degrees */
  public void setPosition(double positionDeg) {
    io.setPosition(positionDeg);
  }

  public void lockTurret(boolean lock) {
    io.lockTurret(lock);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public double getMotorPosition() {
    return inputs.motorPositionDeg;
  }

  public double getPosition() {
    return inputs.turretPositionDeg;
  }
}
