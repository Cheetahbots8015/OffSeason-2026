package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private boolean isRedAlliance = false;
  private boolean inNeutralZone = false;
  private final Translation2d RED_TARGET =
      new Translation2d(
          (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
  private final Translation2d BLUE_TARGET =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance

  private double calculated_angle = 0.0;
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

    double calculated_angle = calculateTurretAngle();
    // setPosition(calculated_angle);
  }

  private double calculateTurretAngle() {
    Translation2d currentPose =
        new Translation2d(
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[0],
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[1]);
    Translation2d offset = new Translation2d(0.122, -0.122);
    currentPose =
        currentPose.plus(
            offset.rotateBy(new Rotation2d(SmartDashboard.getNumber("rotation", 0.0))));

    double shooterPosDegrees = pigeon.getYaw().getValueAsDouble();

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      isRedAlliance = true;
      target = RED_TARGET;
      pigeon_offset = 150;
    } else {
      isRedAlliance = false;
      target = BLUE_TARGET;
      pigeon_offset = -30;
    }
    if (currentPose.getX() < 11.3118646 && currentPose.getX() > 5.229174199999999) {
      inNeutralZone = true;
    } else {
      inNeutralZone = false;
    }
    SmartDashboard.putBoolean("isRed", isRedAlliance);
    SmartDashboard.putBoolean("inNeutral", inNeutralZone);

    double distance =
        Math.sqrt(
                Math.pow(target.getX() - currentPose.getX(), 2)
                    + Math.pow(target.getY() - currentPose.getY(), 2))
            - 0.6036;

    double calculated_difference =
        Math.toDegrees(
                Math.atan2(
                    target.getY()
                        - SmartDashboard.getNumber("xCompensation", 0.0) * Math.pow(distance, 0.5)
                        - currentPose.getY(),
                    target.getX() - currentPose.getX()))
            - shooterPosDegrees
            - pigeon_offset;

    calculated_angle = inputs.turretPositionDeg + calculated_difference;

    calculated_angle += SmartDashboard.getNumber("TurretAngleOffset", 0.0);

    if (calculated_angle > 180) {
      calculated_angle -= 360;
    }
    if (calculated_angle < -180) {
      calculated_angle += 360;
    }
    SmartDashboard.putNumber("calculated_angle", calculated_angle);
    SmartDashboard.putNumber("calculated_difference", calculated_difference);
    return calculated_angle;
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
