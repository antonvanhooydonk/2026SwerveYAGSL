// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/**
 * Constants for the turret subsystem.
 */
public final class TurretConstants {
  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------
  public static final double kTurretGearRatio    =  1.0; // motor shaft rotations per one full 360 deg turret rotation
  public static final double kMinAngleDegrees    = -150.0; // minimum safe raw (unwrapped) turret position, in degrees
  public static final double kMaxAngleDegrees    =  150.0; // maximum safe raw (unwrapped) turret position, in degrees
  
  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kAngleToleranceDegrees         =   1.0;
  public static final double kStallCurrentThreshold         =  25.5; // Amps - 85% of smart current limit
  public static final double kStallVelocityThreshold        =   1.0; // Degrees/sec - indicates motor not moving
  public static final double kMoveTimeoutSeconds            =   3.0; // timeout for moving the turret, in seconds
  public static final double kManualClockwiseVoltage        =  12.0;
  public static final double kManualCounterClockwiseVoltage = -12.0;

  // ------------------------------------------------------------
  // MotionMagic constraints
  // Tuning: start low, increase until motion is fast but smooth
  // ------------------------------------------------------------
  public static final double kCruiseVelocity = 180.0; // rotations per second
  public static final double kAcceleration   = 360.0; // rotations per second squared
  public static final double kJerk           = 3600.0; // rotations per second cubed

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // ------------------------------------------------------------
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
}
