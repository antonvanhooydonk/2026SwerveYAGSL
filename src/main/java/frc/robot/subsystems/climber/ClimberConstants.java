// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/**
 * Constants for the Climber subsystem
 * All values should be tuned based on your specific robot
 */
public final class ClimberConstants {
  // Motor power percentages
  public static final double kUpPercent                 =  0.80;  // Power for climbing up
  public static final double kDownPercent               = -0.80;  // Power for climbing down (negative)
  
  // It is crucial to set these limits correctly to prevent mechanical damage. 
  // Limits should be based on zero being when the climber is straight up.
  // If the climber is zeroed in a different position, then the chain 
  // tensioner may contact the climber gears and cause damage.
  public static final double kUpperLimitDegrees         = -75.0; // Maximum up was -250
  public static final double kLowerLimitDegrees         = 140.0; // Maximum down was 200
  public static final double kHomeDegrees               =   0.0; // Home position
  public static final double kLevelOneClimbDegrees      =   5.0; // Level 1 climb position
  public static final double kLevelTwoClimbDegrees      = -50.0; // Level 2 climb position
  public static final double kPositionToleranceDegrees  =   2.0; // Tolerance for climber positions (in degrees)

  // Stall detection thresholds
  public static final double kStallCurrentThreshold     = 28.0;   // Amps - indicates motor is working hard
  public static final double kStallVelocityThreshold    = 1.0;    // Degrees/sec - indicates motor not moving

  /**
   * Rev through bore encoder v2 resolution (ticks per motor revolution)
   * See: https://revrobotics.ca/rev-11-3174/
   */
  public static final int kEncoderTicksPerRevolution = 8192;
  /**
   * Gear ratio from motor to wheel
   * If motor spins X times, wheel spins 1 time
   * Formula: gear ratio = driven gear teeth / driving gear teeth
   */
  public static final double kGearRatio = 28 / 10;

  /**
   * Position conversion factor: converts encoder ticks to degrees
   * Formula: 360 degrees / gear ratio
   */
  public static final double kPositionConversionFactor = 360.0 / kGearRatio;
  
  /**
   * Velocity conversion factor: converts encoder ticks/minute to degrees/second
   * Formula: position conversion factor / 60 (to convert minutes to seconds)
   */
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
}
