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

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * A standard subsystem that includes an extra periodic callback which runs after the command
 * scheduler. Allows outputs to be published after all other periodic code has finished.
 */
public abstract class FullSubsystem extends SubsystemBase {
  private static final List<FullSubsystem> instances = new ArrayList<>();

  public FullSubsystem() {
    super();
    instances.add(this);
  }

  public FullSubsystem(String name) {
    super(name);
    instances.add(this);
  }

  /**
   * This method is called periodically after the command scheduler, and should be used for applying
   * outputs.
   */
  public abstract void periodicAfterScheduler();

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
