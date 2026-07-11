package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FeederConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem m_subsystem;
  private final IndexerSubsystem m_indexer;
  private final FeederSubsystem m_feeder;
  private final MedianFilter m_filter;
  private double distance;
  private double temp;
  private double shootingSpeedOffset;
  private boolean isNearTrench;
  private boolean isRedAlliance = false;

  // 小巧思
  private XboxController m_Controller;
  private double xCompensation;
  private double yCompensation;

  private final Translation2d RED_TARGET =
      new Translation2d(
          (12.519177399999998 + 11.3118646) / 2, 4.0346376); // Target for Red alliance
  private final Translation2d BLUE_TARGET =
      new Translation2d((5.229174199999999 + 4.0218614) / 2, 4.0346376); // Target for Blue alliance

  private double calculated_angle = 0.0;
  Translation2d target = new Translation2d(0.0, 0.0); // Placeholder for target translation

  public ShootCommand(
      ShooterSubsystem subsystem,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      XboxController controller) {
    m_subsystem = subsystem;
    m_indexer = indexer;
    m_feeder = feeder;
    m_filter = new MedianFilter(3);
    distance = 0;
    temp = 0;
    m_Controller = controller;

    addRequirements(subsystem, indexer, feeder);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    xCompensation = m_Controller.getLeftX() * 0.55;
    yCompensation = m_Controller.getLeftY();
    Translation2d currentPose =
        new Translation2d(
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[0],
            SmartDashboard.getNumberArray("translation", new double[] {0.0, 0.0})[1]);

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      isRedAlliance = true;
      target = RED_TARGET;
    } else {
      isRedAlliance = false;
      target = BLUE_TARGET;
    }
    temp =
        Math.sqrt(
            Math.pow(target.getX() - currentPose.getX(), 2)
                + Math.pow(target.getY() - currentPose.getY(), 2));
    distance = m_filter.calculate(temp - 0.6036);
    SmartDashboard.putNumber("DistanceToTag", distance);
    SmartDashboard.putNumber("PredictedVelocity", Math.sqrt(distance * 14475 + 56439));

    SmartDashboard.putNumber("xCompensation", xCompensation);
    SmartDashboard.putNumber("yCompensation", yCompensation);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorVoltage(0);
    m_feeder.setFeederVoltage(0);
    m_indexer.setMotorVoltage(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
