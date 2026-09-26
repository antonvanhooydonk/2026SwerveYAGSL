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
  // Distance to rotation conversions
  // ============================================================

  /**
   * Converts motor rotations to distance (meters)
   * @param motorRotations Motor position in rotations
   * @param gearRatio Motor rotations per wheel rotation
   * @param circumferenceMeters Circumference in meters
   * @return Wheel distance in meters
   */
  public static double rotationsToMeters(double rotations, double gearRatio, double circumferenceMeters) {
    return (rotations / gearRatio) * circumferenceMeters;
  }

  /**
   * Converts distance (meters) to rotations
   * @param meters Distance in meters
   * @param gearRatio Motor rotations per wheel rotation
   * @param circumferenceMeters Circumference in meters
   * @return Motor position in rotations
   */
  public static double metersToRotations(double meters, double gearRatio, double circumferenceMeters) {
    return (meters / circumferenceMeters) * gearRatio;
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
