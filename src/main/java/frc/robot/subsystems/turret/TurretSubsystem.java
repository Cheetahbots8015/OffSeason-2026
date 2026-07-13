package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private final Pigeon2 pigeon;

  private final Translation2d RED_TARGET =
      new Translation2d(
          (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
  private final Translation2d BLUE_TARGET =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance

  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  private double pigeon_offset = 0.0;

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
    pigeon = new Pigeon2(TurretConstants.kPigeonId);
    pigeon.setYaw(0);
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      target = RED_TARGET;
      pigeon_offset = 150;
    } else {
      target = BLUE_TARGET;
      pigeon_offset = -30;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /** Set turret position in degrees */
  public void setPosition(double positionDeg) {
    io.setPosition(positionDeg);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public double getPosition() {
    return inputs.motorPositionDeg;
  }
}
