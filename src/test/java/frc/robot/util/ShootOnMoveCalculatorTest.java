package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.ShootOnMoveConstants;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;
import org.junit.jupiter.api.Test;

class ShootOnMoveCalculatorTest {

  private static final Translation2d BLUE_TARGET = new Translation2d(4.6255, 4.0346);
  private static final Translation2d RED_TARGET = new Translation2d(11.9155, 4.0346);

  private final ShootOnMoveCalculator calculator =
      new ShootOnMoveCalculator(RED_TARGET, BLUE_TARGET);

  @Test
  void stationaryRobotAimsAtTarget() {
    Pose2d robotPose = new Pose2d(new Translation2d(7.0, 4.0346), new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    ShotParameters params = calculator.calculate(robotPose, speeds);

    assertEquals(BLUE_TARGET.getX(), params.effectiveTarget.getX(), 1e-3);
    assertEquals(BLUE_TARGET.getY(), params.effectiveTarget.getY(), 1e-3);

    // params.turretAngle is field-relative.
    Translation2d turretOffset =
        ShootOnMoveConstants.kTurretOffsetMeters.rotateBy(robotPose.getRotation());
    Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);
    Rotation2d expectedFieldAngle = BLUE_TARGET.minus(turretFieldPos).getAngle();
    assertEquals(expectedFieldAngle.getRadians(), params.turretAngle.getRadians(), 1e-3);

    assertTrue(params.flywheelSpeedRps > 0.0);
  }

  @Test
  void movingForwardShiftsEffectiveTargetBackward() {
    Pose2d robotPose = new Pose2d(new Translation2d(7.0, 4.0346), new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(2.0, 0.0, 0.0);

    ShotParameters params = calculator.calculate(robotPose, speeds);

    // Robot moving +X means the projectile inherits +X velocity, so aim at a target
    // that is shifted -X relative to the real target.
    assertTrue(
        params.effectiveTarget.getX() < BLUE_TARGET.getX(),
        "Effective target should shift opposite to robot velocity");
    assertEquals(BLUE_TARGET.getY(), params.effectiveTarget.getY(), 1e-3);
  }

  @Test
  void fartherDistanceUsesHigherFlywheelSpeed() {
    Pose2d closePose = new Pose2d(new Translation2d(2.0, 4.0346), new Rotation2d());
    Pose2d farPose = new Pose2d(new Translation2d(0.0, 4.0346), new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds();

    double closeSpeed = calculator.calculate(closePose, speeds).flywheelSpeedRps;
    double farSpeed = calculator.calculate(farPose, speeds).flywheelSpeedRps;

    assertTrue(farSpeed > closeSpeed, "Farther shot should use higher flywheel speed");
  }

  @Test
  void turretAngleAccountsForRobotHeading() {
    // Robot at (7, 4.0346), heading 90 deg. Blue target is directly behind in robot frame.
    Pose2d robotPose = new Pose2d(new Translation2d(7.0, 4.0346), Rotation2d.fromDegrees(90.0));
    ChassisSpeeds speeds = new ChassisSpeeds();

    ShotParameters params = calculator.calculate(robotPose, speeds);

    // params.turretAngle is field-relative; compare to the field angle from turret to target.
    Translation2d turretOffset =
        ShootOnMoveConstants.kTurretOffsetMeters.rotateBy(robotPose.getRotation());
    Translation2d turretFieldPos = robotPose.getTranslation().plus(turretOffset);
    Rotation2d expectedFieldAngle = BLUE_TARGET.minus(turretFieldPos).getAngle();

    assertEquals(expectedFieldAngle.getRadians(), params.turretAngle.getRadians(), 1e-3);
  }

  @Test
  void rotationCompensationShiftsEffectiveTargetTangentially() {
    // Robot rotating CCW about its center. Turret offset is (+X, 0), so the turret tangential
    // velocity is in +Y. The effective target must shift in -Y to compensate.
    Pose2d robotPose = new Pose2d(new Translation2d(7.0, 4.0346), new Rotation2d());
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 1.0);

    ShotParameters params = calculator.calculate(robotPose, speeds);

    assertTrue(
        params.effectiveTarget.getY() < BLUE_TARGET.getY(),
        "CCW rotation should shift effective target in -Y when turret is offset +X");
  }

  @Test
  void farShotUsesHigherHoodPosition() {
    // Close shot: should use default hood position
    Pose2d closePose = new Pose2d(new Translation2d(7.0, 4.0346), new Rotation2d());

    // Far shot: distance > switchDistanceMeters
    Pose2d farPose = new Pose2d(new Translation2d(0.0, 0), new Rotation2d());

    ChassisSpeeds speeds = new ChassisSpeeds();

    ShotParameters closeParams = calculator.calculate(closePose, speeds);

    ShotParameters farParams = calculator.calculate(farPose, speeds);

    assertEquals(
        0.0, closeParams.hoodPosition, 1e-3, "Close shots should use default hood position");

    assertEquals(
        ShootOnMoveConstants.hoodPositionOffset,
        farParams.hoodPosition,
        1e-3,
        "Far shots should use higher hood position");

    assertTrue(
        farParams.hoodPosition >= closeParams.hoodPosition,
        "Far shot should have larger hood angle");
  }
}
