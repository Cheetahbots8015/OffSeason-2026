// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.energy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Thermal model of the main breaker using Miner's-rule damage accumulation.
 *
 * <p>Niceness (0–1) scales down the effective trip threshold so the model starts throttling before
 * the actual trip point, providing headroom.
 *
 * <p>Ported from FRC 6328 Mechanical Advantage's 2026 robot code.
 */
public class BreakerModel {
  static final double I_RATED = 120.0;
  private static final double TAU_COOL = 60.0; // Cooldown time constant
  private static final double LOOP_PERIOD_SECS = 0.02;

  // Maximum hold times from breaker datasheet
  private static final double[] I_NORM_PTS = {1.35, 2.0, 2.25, 2.5, 3.0, 4.0, 5.0};
  private static final double[] TRIP_TIME_PTS = {30.0 * 60.0, 70.0, 38.0, 25.0, 15.0, 10.0, 7.0};
  private static final double SENTINEL_TRIP_TIME = 1.0e6;
  private static final double MIN_TRIP_TIME = TRIP_TIME_PTS[TRIP_TIME_PTS.length - 1];

  private static final InterpolatingDoubleTreeMap logTripTimeMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap logInverseTripTimeMap =
      new InterpolatingDoubleTreeMap();

  static {
    logTripTimeMap.put(Math.log(1.0), Math.log(SENTINEL_TRIP_TIME));
    logInverseTripTimeMap.put(Math.log(SENTINEL_TRIP_TIME), Math.log(1.0));
    for (int i = 0; i < I_NORM_PTS.length; i++) {
      logTripTimeMap.put(Math.log(I_NORM_PTS[i]), Math.log(TRIP_TIME_PTS[i]));
      logInverseTripTimeMap.put(Math.log(TRIP_TIME_PTS[i]), Math.log(I_NORM_PTS[i]));
    }
  }

  private final double tripThreshold;
  private double damageState = 0.0;

  BreakerModel(double niceness) {
    tripThreshold = 1.0 - MathUtil.clamp(niceness, 0.0, 1.0);
  }

  public double getDamageState() {
    return damageState;
  }

  double calculateMaxCurrent(double budgetPeriodSecs) {
    double remaining = tripThreshold - damageState;
    if (remaining <= 0.0) {
      return 0.0;
    }

    double requiredTripTime = budgetPeriodSecs / remaining;

    // Edge cases
    if (requiredTripTime >= SENTINEL_TRIP_TIME) {
      return Double.MAX_VALUE;
    }
    if (requiredTripTime <= MIN_TRIP_TIME) {
      return I_NORM_PTS[I_NORM_PTS.length - 1] * I_RATED;
    }

    return Math.exp(logInverseTripTimeMap.get(Math.log(requiredTripTime))) * I_RATED;
  }

  void update(double current) {
    double dt = LOOP_PERIOD_SECS;
    double normalizedI = current / I_RATED;

    boolean cooling;
    if (normalizedI > 1.0) {
      damageState += dt / getTripTime(normalizedI);
      cooling = false;
    } else {
      damageState *= Math.exp(-dt / TAU_COOL);
      cooling = true;
    }
    damageState = MathUtil.clamp(damageState, 0.0, 1.0);

    Logger.recordOutput("BreakerModel/Cooling", cooling);
    Logger.recordOutput("BreakerModel/DamageState", damageState);
  }

  /** Returns the interpolated trip time for the given normalized current. */
  static double getTripTime(double normalizedI) {
    return Math.exp(logTripTimeMap.get(Math.log(Math.max(normalizedI, 1.0))));
  }
}
