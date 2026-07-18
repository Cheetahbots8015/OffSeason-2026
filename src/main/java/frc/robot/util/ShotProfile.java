package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.ShooterConstants;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

final class ShotProfile {
  private final InterpolatingDoubleTreeMap distanceToFlywheelSpeed =
      new InterpolatingDoubleTreeMap();
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
}
