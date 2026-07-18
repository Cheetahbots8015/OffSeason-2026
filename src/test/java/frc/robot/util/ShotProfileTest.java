package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.constants.ShooterConstants;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShotProfileTest {

  @Test
  void fallbackRegressionUsesDefaultSlopeWhenNoData() {
    ShotProfile profile = new ShotProfile();
    profile.fitRegression(new ArrayList<>(), new ArrayList<>());

    double expectedSlope = ShooterConstants.kGear * ShooterConstants.kRadius * Math.PI * 2.0;
    assertEquals(expectedSlope, profile.getSlope(), 1e-9);
    assertEquals(0.0, profile.getIntercept(), 1e-9);
  }

  @Test
  void regressionFitsLinearModel() {
    ShotProfile profile = new ShotProfile();
    // y = 2x + 1
    List<Double> x = List.of(0.0, 1.0, 2.0, 3.0);
    List<Double> y = List.of(1.0, 3.0, 5.0, 7.0);
    profile.fitRegression(x, y);

    assertEquals(2.0, profile.getSlope(), 1e-9);
    assertEquals(1.0, profile.getIntercept(), 1e-9);
  }

  @Test
  void getProjectileSpeedAppliesHoodAngle() {
    ShotProfile profile = new ShotProfile();
    profile.fitRegression(List.of(0.0, 1.0), List.of(0.0, 10.0)); // y = 10x

    double speed = profile.getProjectileSpeed(1.0, 60.0);
    assertEquals(10.0 * Math.cos(Math.toRadians(60.0)), speed, 1e-9);
  }
}
