// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/**
 * Constants for the Climber subsystem
 * All values should be tuned based on your specific robot
 */
public final class ClimberConstants {
  // ------------------------------------------------------------
  // Physical constants - adjust for your robot
  // ------------------------------------------------------------
  // It is crucial to set these limits correctly to prevent mechanical damage. 
  // Limits should be based on zero being when the climber is straight up.
  // If the climber is zeroed in a different position, then the chain 
  // tensioner may contact the climber gears and cause damage.
  public static final int kEncoderTicksPerRevolution    = 8192;
  public static final double kGearRatio                 = 28.0 / 10.0;
  public static final double kMinAngleDegrees           = -75.0;
  public static final double kMaxAngleDegrees           = 140.0;

  // ------------------------------------------------------------
  // Preset angles - adjust for your robot's scoring positions
  // ------------------------------------------------------------
  public static final double kHomeDegrees               =   0.0; 
  public static final double kLevelOneClimbDegrees      =   5.0;
  public static final double kLevelTwoClimbDegrees      = -50.0;

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kAngleToleranceDegrees     =   2.0;
  public static final double kStallCurrentThreshold     =  25.5; // Amps - 85% of smart current limit
  public static final double kStallVelocityThreshold    =   1.0; // Degrees/sec - indicates motor not moving
  public static final double kManualUpVoltage           = -12.0; // manual up control
  public static final double kManualDownVoltage         =  12.0; // manual down control

  // ------------------------------------------------------------
  // Max Motion constraints
  // Tuning: start low, increase until motion is fast but smooth
  // ------------------------------------------------------------
  public static final double kMaxVelocityDegPerSec    = 100.0; // Maximum velocity for position control
  public static final double kMaxAccelDegPerSec2      = 200.0; // Maximum acceleration for position control
  public static final double kMoveTimeoutSeconds      =   3.0; // Timeout for move commands
 
  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // ------------------------------------------------------------
  public static final double kClimberKP = 0.05; // Proportional gain for position control
  public static final double kClimberKI = 0.0; // Integral gain for position control
  public static final double kClimberKD = 0.0; // Derivative gain for position control
}
