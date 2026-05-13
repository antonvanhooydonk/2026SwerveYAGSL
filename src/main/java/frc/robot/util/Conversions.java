// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Utility class for converting between TalonFX (Kraken x60 / Falcon 500) 
 * motor units and real-world swerve drive units.
 *
 * TalonFX native units:
 * - Position: rotations
 * - Velocity: rotations per second (RPS)
 *
 * All methods assume the gear ratio and wheel circumference are passed in,
 * so this class works regardless of the specific swerve module configuration.
 */
public final class Conversions {
  // Prevent instantiation
  private Conversions() {}

  // ============================================================
  // Drive motor conversions
  // ============================================================

  /**
   * Converts drive motor velocity (RPS) to wheel velocity (m/s)
   * @param motorRPS Motor velocity in rotations per second
   * @param gearRatio Motor rotations per wheel rotation
   * @param wheelCircumferenceMeters Wheel circumference in meters
   * @return Wheel velocity in meters per second
   */
  public static double motorRPSToWheelMPS(double motorRPS, double gearRatio, double wheelCircumferenceMeters) {
    return (motorRPS / gearRatio) * wheelCircumferenceMeters;
  }

  /**
   * Converts wheel velocity (m/s) to drive motor velocity (RPS)
   * @param wheelMPS Wheel velocity in meters per second
   * @param gearRatio Motor rotations per wheel rotation
   * @param wheelCircumferenceMeters Wheel circumference in meters
   * @return Motor velocity in rotations per second
   */
  public static double wheelMPSToMotorRPS(double wheelMPS, double gearRatio, double wheelCircumferenceMeters) {
    return (wheelMPS / wheelCircumferenceMeters) * gearRatio;
  }

  /**
   * Converts drive motor position (rotations) to wheel distance (meters)
   * @param motorRotations Motor position in rotations
   * @param gearRatio Motor rotations per wheel rotation
   * @param wheelCircumferenceMeters Wheel circumference in meters
   * @return Wheel distance in meters
   */
  public static double motorRotationsToWheelMeters(double motorRotations, double gearRatio, double wheelCircumferenceMeters) {
    return (motorRotations / gearRatio) * wheelCircumferenceMeters;
  }

  /**
   * Converts wheel distance (meters) to drive motor position (rotations)
   * @param wheelMeters Wheel distance in meters
   * @param gearRatio Motor rotations per wheel rotation
   * @param wheelCircumferenceMeters Wheel circumference in meters
   * @return Motor position in rotations
   */
  public static double wheelMetersToMotorRotations(double wheelMeters, double gearRatio, double wheelCircumferenceMeters) {
    return (wheelMeters / wheelCircumferenceMeters) * gearRatio;
  }

  // ============================================================
  // Steer motor conversions
  // ============================================================

  /**
   * Converts steer motor position (rotations) to wheel angle (radians)
   * @param motorRotations Motor position in rotations
   * @param gearRatio Motor rotations per wheel rotation
   * @return Wheel angle in radians
   */
  public static double motorRotationsToWheelRadians(double motorRotations, double gearRatio) {
    return (motorRotations / gearRatio) * (2 * Math.PI);
  }

  /**
   * Converts wheel angle (radians) to steer motor position (rotations)
   * @param wheelRadians Wheel angle in radians
   * @param gearRatio Motor rotations per wheel rotation
   * @return Motor position in rotations
   */
  public static double wheelRadiansToMotorRotations(double wheelRadians, double gearRatio) {
    return (wheelRadians / (2 * Math.PI)) * gearRatio;
  }

  /**
   * Converts steer motor position (rotations) to wheel angle (degrees)
   * @param motorRotations Motor position in rotations
   * @param gearRatio Motor rotations per wheel rotation
   * @return Wheel angle in degrees
   */
  public static double motorRotationsToWheelDegrees(double motorRotations, double gearRatio) {
    return (motorRotations / gearRatio) * 360.0;
  }

  /**
   * Converts wheel angle (degrees) to steer motor position (rotations)
   * @param wheelDegrees Wheel angle in degrees
   * @param gearRatio Motor rotations per wheel rotation
   * @return Motor position in rotations
   */
  public static double wheelDegreesToMotorRotations(double wheelDegrees, double gearRatio) {
    return (wheelDegrees / 360.0) * gearRatio;
  }

  /**
   * Converts steer motor velocity (RPS) to wheel angular velocity (radians per second)
   * @param motorRPS Motor velocity in rotations per second
   * @param gearRatio Motor rotations per wheel rotation
   * @return Wheel angular velocity in radians per second
   */
  public static double motorRPSToWheelRadiansPerSecond(double motorRPS, double gearRatio) {
    return (motorRPS / gearRatio) * (2 * Math.PI);
  }

  /**
   * Converts wheel angular velocity (radians per second) to steer motor velocity (RPS)
   * @param wheelRadiansPerSecond Wheel angular velocity in radians per second
   * @param gearRatio Motor rotations per wheel rotation
   * @return Motor velocity in rotations per second
   */
  public static double wheelRadiansPerSecondToMotorRPS(double wheelRadiansPerSecond, double gearRatio) {
    return (wheelRadiansPerSecond / (2 * Math.PI)) * gearRatio;
  }

  // ============================================================
  // Angle to rotation conversions
  // ============================================================

  /**
   * Converts rotations to radians, accounting for gear ratio
   * @param rotations Motor position in rotations
   * @param gearRatio Motor rotations per output rotation
   * @return Angle in radians
   */
  public static double rotationsToRadians(double rotations, double gearRatio) {
    return (rotations / gearRatio) * (2 * Math.PI);
  }

  /**
   * Converts radians to rotations, accounting for gear ratio
   * @param radians Angle in radians
   * @param gearRatio Motor rotations per output rotation
   * @return Motor position in rotations
   */
  public static double radiansToRotations(double radians, double gearRatio) {
    return (radians / (2 * Math.PI)) * gearRatio;
  }

  /**
   * Converts rotations to degrees, accounting for gear ratio
   * @param rotations Motor position in rotations
   * @param gearRatio Motor rotations per wheel rotation
   * @return Angle in degrees
   */
  public static double rotationsToDegrees(double rotations, double gearRatio) {
    return (rotations / gearRatio) * 360.0;
  }

  /**
   * Converts degrees to rotations, accounting for gear ratio
   * @param degrees Angle in degrees
   * @param gearRatio Motor rotations per wheel rotation
   * @return Motor position in rotations
   */
  public static double degreesToRotations(double degrees, double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }
}
