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

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Class for logging current, power, and energy usage.
 *
 * <p>Ported from FRC 6328 Mechanical Advantage's 2026 robot code.
 */
public class BatteryLogger {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private double totalCurrent = 0.0;
  private double driveCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  private double batteryVoltage = 12.6;
  private double rioCurrent = 0.0;

  private final Map<String, Double> subsystemCurrents = new HashMap<>();
  private final Map<String, Double> subsystemPowers = new HashMap<>();
  private final Map<String, Double> subsystemEnergies = new HashMap<>();

  public void setBatteryVoltage(double batteryVoltage) {
    this.batteryVoltage = batteryVoltage;
  }

  public void setRioCurrent(double rioCurrent) {
    this.rioCurrent = rioCurrent;
  }

  public double getDriveCurrent() {
    return driveCurrent;
  }

  public void reportCurrentUsage(String key, boolean drive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.abs(amp);
    if (drive) {
      driveCurrent += totalAmps;
    }

    double power = totalAmps * batteryVoltage;
    double energy = power * LOOP_PERIOD_SECS;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);

    String[] keys = key.split("/|-");
    if (keys.length < 2) {
      return;
    }

    String subkey = "";
    for (int i = 0; i < keys.length - 1; i++) {
      subkey += keys[i];
      if (i < keys.length - 2) {
        subkey += "/";
      }
      subsystemCurrents.merge(subkey, totalAmps, Double::sum);
      subsystemPowers.merge(subkey, power, Double::sum);
      subsystemEnergies.merge(subkey, energy, Double::sum);
    }
  }

  public void periodicAfterScheduler() {
    reportCurrentUsage("Controls/roboRIO", false, rioCurrent);
    reportCurrentUsage("Controls/CANcoders", false, 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", false, 0.04);
    reportCurrentUsage("Controls/CANivore", false, 0.03);
    reportCurrentUsage("Controls/Radio", false, 0.5);

    // Log total and subsystem energy usage
    Logger.recordOutput("EnergyLogger/Current", totalCurrent, "amps");
    Logger.recordOutput("EnergyLogger/Power", totalPower, "watts");
    Logger.recordOutput("EnergyLogger/Energy", joulesToWattHours(totalEnergy), "watt hours");

    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue(), "amps");
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue(), "watts");
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy/" + entry.getKey(),
          joulesToWattHours(entry.getValue()),
          "watt hours");
    }
  }

  public void resetTotals() {
    // Reset power and current totals, before next loop
    totalPower = 0.0;
    totalCurrent = 0.0;
    driveCurrent = 0.0;
  }

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  public double getTotalEnergy() {
    return totalEnergy;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}
