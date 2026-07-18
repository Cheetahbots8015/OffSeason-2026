# Shoot-On-The-Move Duplicate-Code Reduction

## Goal

Remove the duplicated regular/far shot logic in `ShootOnMoveCalculator` and the duplicated turret-aim/shooting logic in the three SOTM commands, using overloads and overrides to improve readability and maintainability.

## Scope

In scope:

- `src/main/java/frc/robot/util/ShootOnMoveCalculator.java`
- `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveTriggerCommand.java`
- `src/main/java/frc/robot/commands/ShooterCommands/ShootFarOnMoveTriggerCommand.java`
- `src/main/java/frc/robot/commands/ShooterCommands/ShootOnMoveDefaultCommand.java`
- `src/test/java/frc/robot/util/ShootOnMoveCalculatorTest.java`

Out of scope:

- IO layer / TalonFX boilerplate
- Simple voltage/velocity wrapper commands outside the SOTM flow
- Functional behavior changes beyond the structural refactor

## Background

The calculator currently maintains two parallel sets of fields and methods:

- `distanceToFlywheelSpeed` / `farDistanceToFlywheelSpeed`
- `projectileSpeedSlope` / `farProjectileSpeedSlope`
- `loadCalibrationDataFromFile()` / `loadFarCalibrationDataFromFile()`
- `fitProjectileSpeedRegression()` / `farFitProjectileSpeedRegression()`

The three SOTM commands also duplicate the same pose-to-turret-setpoint math and `wrapTo180` ±170° clamp.

`ShootOnMoveTriggerCommand` and `ShootFarOnMoveTriggerCommand` differ only in:

- Which flywheel speed they command (regular vs. far)
- Hood setpoint (`0` vs. `20`)
- Whether `end()` resets the hood

## Design

### 1. `ShotProfile`

Introduce a package-private class `frc.robot.util.ShotProfile` that encapsulates one shot mode:

```java
final class ShotProfile {
  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeed = new InterpolatingDoubleTreeMap();
  private double projectileSpeedSlope;
  private double projectileSpeedIntercept;

  void loadFromFile(String fileName) { ... }
  void fitRegression(List<Double> flywheelRpsValues, List<Double> projectileSpeedValues) { ... }
  double getFlywheelSpeed(double distanceMeters) { ... }
  double getProjectileSpeed(double flywheelRps, double hoodAngleDeg) { ... }
}
```

- `loadFromFile` is the shared implementation of the current `loadCalibrationDataFromFile` and `loadFarCalibrationDataFromFile` methods; only the filename differs.
- `fitRegression` keeps the existing fallback values when fewer than two points are available.
- `getProjectileSpeed` applies the linear model and the hood-angle cosine factor in one place.

### 2. `ShootOnMoveCalculator`

`ShootOnMoveCalculator` keeps its public constructor and `calculate(Pose2d, ChassisSpeeds)` signature unchanged so callers do not need updates.

Internally it owns two profiles:

```java
private final ShotProfile regularProfile = new ShotProfile();
private final ShotProfile farProfile = new ShotProfile();
```

`populateTables()`:

1. Load `shoot-on-move-data.csv` into `regularProfile` and `shoot-on-move-far-data.csv` into `farProfile`.
2. If a file is missing/empty, fall back to the same sqrt-curve regression for that profile.
3. Fit each profile's regression from its loaded/generated data.

`calculate()`:

- Runs the same convergence loop.
- Uses `regularProfile.getFlywheelSpeed(distance)` and `regularProfile.getProjectileSpeed(...)` for the regular shot.
- Uses `farProfile.getFlywheelSpeed(distance)` and `farProfile.getProjectileSpeed(...)` for the far shot.
- Continues to use `distance` (not `fardistance`) for the far flywheel lookup, matching the current intentional behavior.
- Returns the existing `ShotParameters` record with both `flywheelSpeedRps` and `farFlyWheelSpeedRps`.

### 3. Command hierarchy

Introduce an abstract base class `ShootOnMoveAimCommand` in `frc.robot.commands.ShooterCommands`:

```java
public abstract class ShootOnMoveAimCommand extends Command {
  protected final Drive drive;
  protected final TurretSubsystem turret;
  protected final ShootOnMoveCalculator calculator;

  protected ShootOnMoveAimCommand(Drive drive, TurretSubsystem turret, ShootOnMoveCalculator calculator) {
    this.drive = drive;
    this.turret = turret;
    this.calculator = calculator;
    addRequirements(turret);
  }

  protected final double computeAim() { ... }

  protected abstract void onAim(double turretSetpoint);
}
```

The base class `execute()` method does:

```java
@Override
public void execute() {
  double turretSetpoint = computeAim();
  turret.setPosition(turretSetpoint);
  onAim(turretSetpoint);
}
```

`computeAim()` performs the shared work:

- Convert robot-relative chassis speeds to field-relative.
- Call `calculator.calculate(drive.getPose(), fieldRelativeSpeeds)`.
- Compute `turretSetpointDeg` and apply `wrapTo180` plus the ±170° clamp.
- Record the shared `Logger` outputs.
- Return the clamped turret setpoint in degrees.

Subclasses:

- `ShootOnMoveDefaultCommand extends ShootOnMoveAimCommand`
  - Constructor: `ShootOnMoveDefaultCommand(Drive, TurretSubsystem, HoodSubsystem, ShootOnMoveCalculator)`.
  - `onAim(double)` is empty; the base class already aims the turret.
  - Keeps the current `addRequirements(turret)` only.

- `ShootOnMoveTriggerCommand extends ShootOnMoveAimCommand`
  - Constructor adds `HoodSubsystem`, `ShooterSubsystem`, `IndexerSubsystem`, `FeederSubsystem`.
  - Overrides `onAim(double)` to command hood, shooter, indexer, and feeder when aligned, after the base class has already set the turret position.
  - Introduces protected hooks so the far variant can override without copying the body:
    - `protected double getFlywheelSpeedRps(ShotParameters params)` → default `params.flywheelSpeedRps`
    - `protected double getHoodPositionDeg()` → default `0.0`
    - `protected void onEndShooting()` → default stops shooter, indexer, and feeder

- `ShootFarOnMoveTriggerCommand extends ShootOnMoveTriggerCommand`
  - Overrides `getFlywheelSpeedRps(...)` → `params.farFlyWheelSpeedRps`
  - Overrides `getHoodPositionDeg()` → `20.0`
  - Overrides `onEndShooting()` → stops shooter/indexer/feeder and resets hood to `0.0`

`RobotContainer` construction and button bindings remain unchanged because the public constructors of the three concrete commands stay compatible.

## Behavior Preservation

- `ShootOnMoveCalculator` public API is unchanged.
- `ShotParameters` fields and semantics are unchanged.
- File-load and regression fallbacks are unchanged.
- Command `addRequirements` lists are unchanged.
- Far flywheel lookup in the convergence loop continues to use `distance`, as confirmed by the team.

## Testing Plan

1. Run `./gradlew build` (or Windows equivalent) to confirm compilation and existing tests.
2. Existing `ShootOnMoveCalculatorTest` must pass without modification.
3. Add focused tests for `ShotProfile` if package visibility allows:
   - Regression fallback when fewer than two points are provided.
   - `getProjectileSpeed` applies the hood-angle cosine.
4. Manually verify in simulator or on-robot that regular and far SOTM commands still command the expected hood setpoints and flywheel speeds.

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| `ShotProfile` changes the logging key names under `ShootOnMove/` | Keep the same `Logger.recordOutput` keys in `populateTables()` and `calculate()`. |
| Command subclass requirements differ from base class | Only the base constructor adds `turret`; subclasses add their extra subsystems in their own constructors. |
| Inheritance makes the far command harder to understand | The far command is only three one-line overrides; the shared logic lives in one place. |
