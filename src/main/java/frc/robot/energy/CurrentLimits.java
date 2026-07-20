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

/** Current-limit constants used by the {@link FinanceDepartment} energy budget. Tunable. */
public final class CurrentLimits {
  public static final double driveMinLimitAmps = 10.0;
  public static final double driveMaxLimitAmps = 40.0;
  public static final double driveAutoLimitAmps = 60.0;
  public static final double driveProbeRateBrownout = 50.0; // Amps/second

  private CurrentLimits() {}
}
