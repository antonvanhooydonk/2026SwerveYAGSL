// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public final class Utils {
  // Prevent instantiation
  private Utils() {}

  /**
   * Log robot info message to console or other notification systems.
   */
  public static void logInfo(String msg) {
    System.out.println(msg);
  }

  /**
   * Log robot error message to console or other notification systems.
   */
  public static void logError(String msg) {
    System.err.println(msg);
    DriverStation.reportError(msg, false);
    Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Error", msg));
  }

  /**
   * Determine if the robot is currently on the red alliance.
   * @return True if the robot is on the red alliance, false otherwise.
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Normalizes an angle to the range (-π, π)
   * @param angleRadians Angle in radians
   * @return Normalized angle in radians
   */
  public static double normalizeAngle(double angleRadians) {
    return MathUtil.inputModulus(angleRadians, -Math.PI, Math.PI);
  }

  /**
   * Rounds a double value to 4 decimal places for display in telemetry dashboards.
   * Do not use for precise calculations internally!
   * @param value The double value to format.
   * @return The double value formatted to 4 decimal places.
   */
  public static double showDouble(double value) {
    return Math.round(value * 10000.0) / 10000.0;
  }
}
