# SOTM Duplicate-Code Reduction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace duplicated regular/far shot logic in `ShootOnMoveCalculator` and duplicated turret-aim logic in the three SOTM commands with a shared `ShotProfile` class and a `ShootOnMoveAimCommand` hierarchy.

**Architecture:** A package-private `ShotProfile` encapsulates one shot mode's interpolation table and regression. `ShootOnMoveCalculator` owns two profiles. An abstract `ShootOnMoveAimCommand` performs the shared pose-to-turret-setpoint math; concrete subclasses add their own shooting behavior, and the far command overrides three protected hooks.

**Tech Stack:** Java 17, WPILib 2025, CTRE Phoenix 6, JUnit 5, AdvantageKit.

## Global Constraints

- Preserve the public API of `ShootOnMoveCalculator` and the public constructors of the three concrete SOTM commands.
- Preserve all existing logging keys under `ShootOnMove/`.
- Preserve file-load and regression fallback behavior exactly.
- Far flywheel lookup in the convergence loop must continue to use `distance`, not `fardistance`.
- All tests in `ShootOnMoveCalculatorTest` must pass without modification.
- Build must pass with `./gradlew build` (or Windows equivalent).

---

## Task 1: Create `ShotProfile` class with tests

**Files:**
- Create: `src/main/java/frc/robot/util/ShotProfile.java`
- Create: `src/test/java/frc/robot/util/ShotProfileTest.java`

**Interfaces:**
- Consumes: constants from `ShooterConstants`.
- Produces: `ShotProfile` with `List<double[]> loadFromFile(String, String)`, `void generateFallbackData(double, double, double, double)`, `void fitRegression(List<Double>, List<Double>)`, `double getFlywheelSpeed(double)`, `double getProjectileSpeed(double, double)`, `double getSlope()`, `double getIntercept()`.

- [ ] **Step 1: Create the test file**

Create `src/test/java/frc/robot/util/ShotProfileTest.java`:

```java
package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

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
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
./gradlew test --tests frc.robot.util.ShotProfileTest
```

Expected: compilation fails because `ShotProfile` and `getSlope`/`getIntercept` do not exist.

- [ ] **Step 3: Create `ShotProfile` skeleton**

Create `src/main/java/frc/robot/util/ShotProfile.java`:

```java
package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.ShooterConstants;
import java.util.List;

final class ShotProfile {
  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private double projectileSpeedSlope;
  private double projectileSpeedIntercept;

  void fitRegression(List<Double> flywheelRpsValues, List<Double> projectileSpeedValues) {
    int n = flywheelRpsValues.size();
    if (n < 2) {
      projectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius * Math.PI * 2.0;
      projectileSpeedIntercept = 0.0;
      return;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = flywheelRpsValues.get(i);
      double y = projectileSpeedValues.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }

    double denominator = n * sumX2 - sumX * sumX;
    if (Math.abs(denominator) < 1e-9) {
      projectileSpeedSlope = ShooterConstants.kGear * ShooterConstants.kRadius;
      projectileSpeedIntercept = 0.0;
      return;
    }

    projectileSpeedSlope = (n * sumXY - sumX * sumY) / denominator;
    projectileSpeedIntercept = (sumY - projectileSpeedSlope * sumX) / n;
  }

  double getProjectileSpeed(double flywheelRps, double hoodAngleDeg) {
    double speed = projectileSpeedSlope * flywheelRps + projectileSpeedIntercept;
    return speed * Math.cos(Math.toRadians(hoodAngleDeg));
  }

  double getSlope() {
    return projectileSpeedSlope;
  }

  double getIntercept() {
    return projectileSpeedIntercept;
  }
}
```

- [ ] **Step 4: Add file loading and fallback data generation**

Append to `ShotProfile`:

```java
  List<double[]> loadFromFile(String fileName, String logKey) {
    List<double[]> data = new ArrayList<>();
    File file = new File(Filesystem.getDeployDirectory(), fileName);
    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
      String line;
      boolean firstLine = true;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) {
          continue;
        }
        if (firstLine) {
          firstLine = false;
          continue;
        }
        String[] parts = line.split(",");
        if (parts.length >= 3) {
          double distanceMeters = Double.parseDouble(parts[0].trim());
          double flywheelRps = Double.parseDouble(parts[1].trim());
          double projectileSpeedMps = Double.parseDouble(parts[2].trim());
          data.add(new double[] {distanceMeters, flywheelRps, projectileSpeedMps});
        }
      }
      Logger.recordOutput(
          "ShootOnMove/" + logKey + "DataLoaded", "Loaded " + data.size() + " entries from file");
    } catch (Exception e) {
      Logger.recordOutput(
          "ShootOnMove/" + logKey + "DataLoadError",
          "Failed to load " + file.getAbsolutePath() + ": " + e.getMessage());
    }

    for (double[] row : data) {
      distanceToFlywheelSpeed.put(row[0], row[1]);
    }
    return data;
  }

  void generateFallbackData(double slope, double intercept, double gear, double radius) {
    for (double distanceMeters = 1.0; distanceMeters <= 8.0; distanceMeters += 0.5) {
      double flywheelRps = Math.sqrt(distanceMeters * slope + intercept) / (2.0 * Math.PI);
      double projectileSpeedMps = flywheelRps * gear * radius * 2 * Math.PI;
      distanceToFlywheelSpeed.put(distanceMeters, flywheelRps);
    }
  }

  double getFlywheelSpeed(double distanceMeters) {
    return distanceToFlywheelSpeed.get(distanceMeters);
  }
```

Add imports:

```java
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
```

- [ ] **Step 5: Run tests**

Run:

```bash
./gradlew test --tests frc.robot.util.ShotProfileTest
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/util/ShotProfile.java src/test/java/frc/robot/util/ShotProfileTest.java
git commit -m "Add ShotProfile class for shared SOTM shot-mode logic"
```

---

## Task 2: Refactor `ShootOnMoveCalculator` to use `ShotProfile`

**Files:**
- Modify: `src/main/java/frc/robot/util/ShootOnMoveCalculator.java`

**Interfaces:**
- Consumes: `ShotProfile` from Task 1.
- Produces: `ShootOnMoveCalculator` with same public constructor and `calculate()` signature; internally uses `regularProfile` and `farProfile`.

- [ ] **Step 1: Run existing calculator tests as a baseline**

Run:

```bash
./gradlew test --tests frc.robot.util.ShootOnMoveCalculatorTest
```

Expected: all tests pass.

- [ ] **Step 2: Replace duplicated fields with two profiles**

In `ShootOnMoveCalculator`, replace:

```java
  // Distance (meters) -> flywheel speed (rotations per second)
  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeed =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap farDistanceToFlywheelSpeed =
      new InterpolatingDoubleTreeMap();

  // Linear model: projectileSpeedMps = m * flywheelRps + b
  private double projectileSpeedSlope;
  private double projectileSpeedIntercept;
  private double farProjectileSpeedSlope;
  private double farProjectileSpeedIntercept;
```

with:

```java
  private final ShotProfile regularProfile = new ShotProfile();
  private final ShotProfile farProfile = new ShotProfile();
```

Remove the now-unused `InterpolatingDoubleTreeMap` import if it is no longer used elsewhere.

- [ ] **Step 3: Rewrite `populateTables()` to use profiles**

Replace the entire `populateTables()` method with:

```java
  private void populateTables() {
    List<double[]> calibrationData =
        regularProfile.loadFromFile("shoot-on-move-data.csv", "Calibration");
    List<double[]> farCalibrationData =
        farProfile.loadFromFile("shoot-on-move-far-data.csv", "FarCalibration");

    if (calibrationData.isEmpty()) {
      regularProfile.generateFallbackData(
          ShootOnMoveConstants.kFlywheelCurveSlope,
          ShootOnMoveConstants.kFlywheelCurveIntercept,
          ShooterConstants.kGear,
          ShooterConstants.kRadius);
    }

    if (farCalibrationData.isEmpty()) {
      farProfile.generateFallbackData(
          ShootOnMoveConstants.kFlywheelCurveSlope,
          ShootOnMoveConstants.kFlywheelCurveIntercept,
          ShooterConstants.kGear,
          ShooterConstants.kRadius);
    }

    List<Double> flywheelRpsValues = new ArrayList<>();
    List<Double> projectileSpeedValues = new ArrayList<>();
    for (double[] row : calibrationData) {
      flywheelRpsValues.add(row[1]);
      projectileSpeedValues.add(row[2]);
    }
    regularProfile.fitRegression(flywheelRpsValues, projectileSpeedValues);

    List<Double> farFlywheelRpsValues = new ArrayList<>();
    List<Double> farProjectileSpeedValues = new ArrayList<>();
    for (double[] row : farCalibrationData) {
      farFlywheelRpsValues.add(row[1]);
      farProjectileSpeedValues.add(row[2]);
    }
    farProfile.fitRegression(farFlywheelRpsValues, farProjectileSpeedValues);

    Logger.recordOutput("ShootOnMove/ProjectileSpeedSlope", regularProfile.getSlope());
    Logger.recordOutput("ShootOnMove/ProjectileSpeedIntercept", regularProfile.getIntercept());
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedSlope", farProfile.getSlope());
    Logger.recordOutput("ShootOnMove/FarProjectileSpeedIntercept", farProfile.getIntercept());
  }
```

- [ ] **Step 4: Remove old file loaders and regression methods**

Delete these methods from `ShootOnMoveCalculator`:

- `loadCalibrationDataFromFile()`
- `loadFarCalibrationDataFromFile()`
- `fitProjectileSpeedRegression(...)`
- `farFitProjectileSpeedRegression(...)`

Also remove any imports that are no longer used (e.g., `BufferedReader`, `File`, `FileReader` if they are only used by the deleted loaders).

- [ ] **Step 5: Update `calculate()` to use profiles**

In `calculate()`, replace regular/far flywheel and projectile-speed lookups:

```java
      double flywheelSpeed = regularProfile.getFlywheelSpeed(distance);
      double farflywheelSpeed = farProfile.getFlywheelSpeed(distance);

      double projectileSpeed =
          regularProfile.getProjectileSpeed(flywheelSpeed, ShootOnMoveConstants.hoodDefaultPosition);
      double farprojectileSpeed =
          farProfile.getProjectileSpeed(
              farflywheelSpeed, ShootOnMoveConstants.hoodDefaultPosition - ShootOnMoveConstants.hoodPositionOffset);
```

At the end of `calculate()`, replace:

```java
    double flywheelSpeed = distanceToFlywheelSpeed.get(finalDistance);
    double farflywheelSpeed = farDistanceToFlywheelSpeed.get(farFinalDistance);
```

with:

```java
    double flywheelSpeed = regularProfile.getFlywheelSpeed(finalDistance);
    double farflywheelSpeed = farProfile.getFlywheelSpeed(farFinalDistance);
```

- [ ] **Step 6: Run existing calculator tests**

Run:

```bash
./gradlew test --tests frc.robot.util.ShootOnMoveCalculatorTest
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/main/java/frc/robot/util/ShootOnMoveCalculator.java src/main/java/frc/robot/util/ShotProfile.java
git commit -m "Refactor ShootOnMoveCalculator to use ShotProfile"
```

---

## Task 3: Create `ShootOnMoveAimCommand` base class

**Files:**
- Create: `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveAimCommand.java`

**Interfaces:**
- Consumes: `Drive`, `TurretSubsystem`, `ShootOnMoveCalculator`, `ChassisSpeeds`, `Logger`.
- Produces: abstract `ShootOnMoveAimCommand` with `computeAim()` returning `AimResult` and `onAim(AimResult)`.

- [ ] **Step 1: Create the base class**

Create `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveAimCommand.java`:

```java
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;
import org.littletonrobotics.junction.Logger;

public abstract class ShootOnMoveAimCommand extends Command {
  protected final Drive drive;
  protected final TurretSubsystem turret;
  protected final ShootOnMoveCalculator calculator;

  protected record AimResult(double turretSetpointDeg, ShotParameters parameters) {}

  protected ShootOnMoveAimCommand(
      Drive drive, TurretSubsystem turret, ShootOnMoveCalculator calculator) {
    this.drive = drive;
    this.turret = turret;
    this.calculator = calculator;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    AimResult result = computeAim();
    turret.setPosition(result.turretSetpointDeg());
    onAim(result);
  }

  protected final AimResult computeAim() {
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, drive.getRotation());
    ShotParameters params = calculator.calculate(drive.getPose(), fieldRelativeSpeeds);

    double turretSetpointDeg = params.turretAngle.minus(drive.getRotation()).getDegrees();
    double turretOffset = 60;
    double turretSetpoint = wrapTo180(turretSetpointDeg + turretOffset);
    if (turretSetpoint > 170) {
      turretSetpoint = 170;
    } else if (turretSetpoint < -170) {
      turretSetpoint = -170;
    }

    Logger.recordOutput("ShootOnMove/EffectiveTarget", params.effectiveTarget);
    Logger.recordOutput("ShootOnMove/TurretSetpointDeg", turretSetpointDeg);
    Logger.recordOutput("ShootOnMove/FinalDegree", turretSetpoint);
    Logger.recordOutput(
        "ShootOnMove/ShooterSetpointRadPerSec", params.flywheelSpeedRps * 2.0 * Math.PI);
    Logger.recordOutput(
        "ShootOnMove/FarShooterSetpointRadPerSec", params.farFlyWheelSpeedRps * 2.0 * Math.PI);
    Logger.recordOutput("ShootOnMove/FieldTurretAngleRad", params.turretAngle.getRadians());

    return new AimResult(turretSetpoint, params);
  }

  protected abstract void onAim(AimResult result);

  private double wrapTo180(double degrees) {
    while (degrees > 180.0) {
      degrees -= 360.0;
    }
    while (degrees < -180.0) {
      degrees += 360.0;
    }
    return degrees;
  }
}
```

- [ ] **Step 2: Compile to check for errors**

Run:

```bash
./gradlew compileJava
```

Expected: compilation succeeds (the class is not yet used).

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveAimCommand.java
git commit -m "Add ShootOnMoveAimCommand base class"
```

---

## Task 4: Refactor `ShootOnMoveDefaultCommand` to extend base class

**Files:**
- Modify: `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveDefaultCommand.java`

**Interfaces:**
- Consumes: `ShootOnMoveAimCommand` from Task 3.
- Produces: `ShootOnMoveDefaultCommand` extending `ShootOnMoveAimCommand`, constructor signature unchanged.

- [ ] **Step 1: Rewrite the command**

Replace the entire content of `ShootOnMoveDefaultCommand.java` with:

```java
package frc.robot.commands.ShooterCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;

public class ShootOnMoveDefaultCommand extends ShootOnMoveAimCommand {
  private final HoodSubsystem hood;

  public ShootOnMoveDefaultCommand(
      Drive drive, TurretSubsystem turret, HoodSubsystem hood, ShootOnMoveCalculator calculator) {
    super(drive, turret, calculator);
    this.hood = hood;
  }

  @Override
  protected void onAim(AimResult result) {
    // Default command only aims the turret; hood control is handled by trigger commands.
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
```

- [ ] **Step 2: Compile**

Run:

```bash
./gradlew compileJava
```

Expected: compilation succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveDefaultCommand.java
git commit -m "Refactor ShootOnMoveDefaultCommand to extend ShootOnMoveAimCommand"
```

---

## Task 5: Refactor `ShootOnMoveTriggerCommand` to extend base class

**Files:**
- Modify: `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveTriggerCommand.java`

**Interfaces:**
- Consumes: `ShootOnMoveAimCommand` from Task 3.
- Produces: `ShootOnMoveTriggerCommand` extending `ShootOnMoveAimCommand`, with protected hooks `getFlywheelSpeedRps`, `getHoodPositionDeg`, `onEndShooting`.

- [ ] **Step 1: Rewrite the command**

Replace the entire content of `ShootOnMoveTriggerCommand.java` with:

```java
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.CheetahUtil;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;

public class ShootOnMoveTriggerCommand extends ShootOnMoveAimCommand {
  protected final HoodSubsystem hood;
  protected final ShooterSubsystem shooter;
  protected final IndexerSubsystem indexer;
  protected final FeederSubsystem feeder;

  public ShootOnMoveTriggerCommand(
      Drive drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShootOnMoveCalculator calculator) {
    super(drive, turret, calculator);
    this.hood = hood;
    this.shooter = shooter;
    this.indexer = indexer;
    this.feeder = feeder;
    addRequirements(hood, shooter, indexer, feeder);
  }

  @Override
  protected void onAim(AimResult result) {
    double turretSetpoint = result.turretSetpointDeg();
    ShotParameters params = result.parameters();

    double shooterSetpointRadPerSec = getFlywheelSpeedRps(params) * 2.0 * Math.PI;
    shooter.VelocityVoltage(shooterSetpointRadPerSec + SmartDashboard.getNumber("shooterOffset", 0));
    hood.setPosition(getHoodPositionDeg());

    if (CheetahUtil.isNear(turret.getPosition(), turretSetpoint, 10)
        && CheetahUtil.isNear(
            shooterSetpointRadPerSec + SmartDashboard.getNumber("shooterOffset", 0),
            shooter.getMotorVelocity(),
            10)) {
      indexer.VelocityVoltage(
          SmartDashboard.getNumber("indexerHorizontalVelocity", 0),
          SmartDashboard.getNumber("indexerVerticleVelocity", 0));
      feeder.setFeederVelocityVoltage(SmartDashboard.getNumber("feederVelocity", 0));
    } else {
      indexer.setMotorVoltage(0, 0);
      feeder.setFeederVoltage(0);
    }
  }

  protected double getFlywheelSpeedRps(ShotParameters params) {
    return params.flywheelSpeedRps;
  }

  protected double getHoodPositionDeg() {
    return 0.0;
  }

  @Override
  public void end(boolean interrupted) {
    onEndShooting();
  }

  protected void onEndShooting() {
    shooter.setMotorVoltage(0.0);
    indexer.setMotorVoltage(0, 0);
    feeder.setFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
```

- [ ] **Step 2: Compile**

Run:

```bash
./gradlew compileJava
```

Expected: compilation succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveTriggerCommand.java
git commit -m "Refactor ShootOnMoveTriggerCommand to extend ShootOnMoveAimCommand"
```

---

## Task 6: Refactor `ShootFarOnMoveTriggerCommand` to extend `ShootOnMoveTriggerCommand`

**Files:**
- Modify: `src/main/java/frc/robot/commands/ShooterCommands/ShootFarOnMoveTriggerCommand.java`

**Interfaces:**
- Consumes: `ShootOnMoveTriggerCommand` from Task 5.
- Produces: `ShootFarOnMoveTriggerCommand` extending `ShootOnMoveTriggerCommand`, only three overrides.

- [ ] **Step 1: Rewrite the command**

Replace the entire content of `ShootFarOnMoveTriggerCommand.java` with:

```java
package frc.robot.commands.ShooterCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShootOnMoveCalculator;
import frc.robot.util.ShootOnMoveCalculator.ShotParameters;

public class ShootFarOnMoveTriggerCommand extends ShootOnMoveTriggerCommand {

  public ShootFarOnMoveTriggerCommand(
      Drive drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      FeederSubsystem feeder,
      ShootOnMoveCalculator calculator) {
    super(drive, turret, hood, shooter, indexer, feeder, calculator);
  }

  @Override
  protected double getFlywheelSpeedRps(ShotParameters params) {
    return params.farFlyWheelSpeedRps;
  }

  @Override
  protected double getHoodPositionDeg() {
    return 20.0;
  }

  @Override
  protected void onEndShooting() {
    super.onEndShooting();
    hood.setPosition(0);
  }
}
```

- [ ] **Step 2: Compile**

Run:

```bash
./gradlew compileJava
```

Expected: compilation succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/commands/ShooterCommands/ShootFarOnMoveTriggerCommand.java
git commit -m "Refactor ShootFarOnMoveTriggerCommand to extend ShootOnMoveTriggerCommand"
```

---

## Task 7: Final verification

**Files:**
- All files from Tasks 1-6.

- [ ] **Step 1: Run all unit tests**

Run:

```bash
./gradlew test
```

Expected: all tests pass, including `ShootOnMoveCalculatorTest` and `ShotProfileTest`.

- [ ] **Step 2: Run full build**

Run:

```bash
./gradlew build
```

Expected: build succeeds.

- [ ] **Step 3: Review git diff**

Run:

```bash
git diff --stat
```

Expected: only the intended SOTM files are modified.

- [ ] **Step 4: Final commit if any uncommitted changes remain**

```bash
git status
# If there are changes:
git add -A
git commit -m "Complete SOTM duplicate-code reduction"
```

---

## Self-Review Checklist

- [ ] Spec coverage: `ShotProfile`, calculator refactor, command hierarchy, behavior preservation, and testing are all represented.
- [ ] No placeholders: every step has concrete code, exact commands, and expected outcomes.
- [ ] Type consistency: `ShotProfile` API matches across tasks; command constructor signatures match the spec.
- [ ] DRY: shared logic lives in `ShotProfile` and `ShootOnMoveAimCommand`; far command only overrides hooks.
