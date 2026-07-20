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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Central energy manager ("finance department" for power): collects per-subsystem current reports,
 * models the battery and main breaker, computes the robot-wide current budget, and allocates the
 * drive supply-current limit.
 *
 * <p>Subsystems report their supply-current draw every loop via {@link #reportCurrentUsage}; after
 * the command scheduler runs, {@link #periodicAfterScheduler()} aggregates the reports, updates the
 * models, and publishes budgets and alerts.
 *
 * <p>Ported from FRC 6328 Mechanical Advantage's 2026 robot code.
 */
public class FinanceDepartment extends FullSubsystem {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double minVoltageBrownout = 7.0;
  private static final double maxBudgetAmps = 200.0;
  private static final double breakerNiceness = 0.05;
  private static final double budgetWarningThreshold = 180.0;
  // Allow ramping from other subsystems
  private static final double budgetHeadroom = 0.9;
  // Time we can run max budget before trip
  private static final double breakerDangerHorizonSecs = 3.0;
  private final double breakerDamageWarningThreshold;

  private static final Alert budgetWarning =
      new Alert("Battery is low, robot performance may be degraded.", AlertType.kInfo);
  private static final Debouncer budgetWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);
  private static final Alert brownoutWarning =
      new Alert("Brownout detected, drive performance may be degraded.", AlertType.kWarning);
  private static final Alert breakerDamageWarning =
      new Alert("Breaker damage is high, please stop using the robot.", AlertType.kWarning);
  private static final Debouncer breakerDamageWarningDebouncer =
      new Debouncer(0.5, DebounceType.kBoth);

  private static FinanceDepartment instance;

  public static FinanceDepartment getInstance() {
    if (instance == null) instance = new FinanceDepartment();
    return instance;
  }

  private final BatteryLogger energyLogger = new BatteryLogger();
  private final BatteryEstimator battery = new BatteryEstimator();
  private final BreakerModel breaker = new BreakerModel(breakerNiceness);
  private final BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  private double budget = 0.0;
  private double driveBudget = 0.0;
  private final Debouncer brownoutDebouncer = new Debouncer(2.0, DebounceType.kFalling);

  private FinanceDepartment() {
    // Solve for damage state where breaker will trip if we run at maxBudgetAmps for horizon.
    breakerDamageWarningThreshold =
        (1.0 - breakerNiceness)
            - (breakerDangerHorizonSecs
                / BreakerModel.getTripTime(maxBudgetAmps / BreakerModel.I_RATED));
  }

  public void reset() {
    battery.setInitialVoltage(inputs.batteryVoltage, energyLogger.getTotalCurrent());
  }

  public void reportCurrentUsage(String key, boolean drive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.max(0.0, amp);
    energyLogger.reportCurrentUsage(key, drive, totalAmps);
  }

  @Override
  public void periodic() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.rioCurrent = RobotController.getInputCurrent();
    inputs.brownedOut = RobotController.isBrownedOut();
    Logger.processInputs("EnergyLogger", inputs);
    energyLogger.setBatteryVoltage(inputs.batteryVoltage);
    energyLogger.setRioCurrent(inputs.rioCurrent);
  }

  @Override
  public void periodicAfterScheduler() {
    // Run energy logger
    energyLogger.periodicAfterScheduler();

    // Update models
    battery.update(energyLogger.getTotalCurrent(), inputs.batteryVoltage);
    breaker.update(energyLogger.getTotalCurrent());

    // Calculate budgets
    double batteryMaxCurrent = battery.calculateMaxCurrent(minVoltageBrownout);
    double breakerMaxCurrent = breaker.calculateMaxCurrent(breakerDangerHorizonSecs);
    Logger.recordOutput("FinanceDepartment/BatteryMaxCurrent", batteryMaxCurrent);
    Logger.recordOutput("FinanceDepartment/BreakerMaxCurrent", breakerMaxCurrent);
    budget =
        Math.min(Math.min(batteryMaxCurrent, breakerMaxCurrent) * budgetHeadroom, maxBudgetAmps);

    boolean brownoutDebounced = brownoutDebouncer.calculate(inputs.brownedOut);
    if (!brownoutDebounced) {
      driveBudget = budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
    } else {
      double calculatedBudget =
          budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
      // Asymmetric ramping of drive budget
      driveBudget =
          calculatedBudget < driveBudget
              ? calculatedBudget
              : Math.min(
                  calculatedBudget,
                  driveBudget + CurrentLimits.driveProbeRateBrownout * LOOP_PERIOD_SECS);
    }
    driveBudget = Math.max(0.0, driveBudget);

    Logger.recordOutput("FinanceDepartment/Budget", budget);
    Logger.recordOutput("FinanceDepartment/DriveBudget", driveBudget);

    // Update alerts
    budgetWarning.set(budgetWarningDebouncer.calculate(budget < budgetWarningThreshold));
    brownoutWarning.set(brownoutDebounced);
    breakerDamageWarning.set(
        breakerDamageWarningDebouncer.calculate(
            breaker.getDamageState() > breakerDamageWarningThreshold));

    energyLogger.resetTotals();
  }

  /** Get the per-module drive supply current limit from the current energy budget. */
  public double getDriveLimit() {
    double driveLimit =
        Math.floor(
                MathUtil.clamp(
                        driveBudget / 4.0,
                        CurrentLimits.driveMinLimitAmps,
                        CurrentLimits.driveMaxLimitAmps)
                    / 0.5)
            * 0.5;
    if (DriverStation.isAutonomous()) {
      driveLimit = CurrentLimits.driveAutoLimitAmps;
    }
    Logger.recordOutput("FinanceDepartment/DriveLimit", driveLimit);
    return driveLimit;
  }

  public double getBatteryVoltage() {
    return inputs.batteryVoltage;
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
    public boolean brownedOut = false;
  }
}
