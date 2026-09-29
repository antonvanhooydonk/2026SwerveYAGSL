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
  public static final double kPositionConversionFactor  = 360.0 / kGearRatio;
  public static final double kVelocityConversionFactor  = kPositionConversionFactor / 60.0;
  public static final double kUpperLimitDegrees         = -75.0; // Maximum up was -250
  public static final double kLowerLimitDegrees         = 140.0; // Maximum down was 200

  // "Up" is the NEGATIVE encoder direction on this robot, so the upper limit is numerically 
  // smaller than the lower limit. Anything that needs numeric min/max (clamp, soft limits) 
  // must use these, never upper/lower directly - MathUtil.clamp(x, lower, upper) would 
  // always return the lower limit here.
  public static final double kMinPositionDegrees        = Math.min(kUpperLimitDegrees, kLowerLimitDegrees);
  public static final double kMaxPositionDegrees        = Math.max(kUpperLimitDegrees, kLowerLimitDegrees);

  // Sign of the encoder change when moving "up": -1 here. Assumes POSITIVE motor voltage 
  // increases the encoder. Verify on the bench with the climber unloaded; if it doesn't, 
  // invert the motor with .inverted(true) in configureMotor() rather than changing this.
  public static final double kUpDirection               = Math.signum(kUpperLimitDegrees - kLowerLimitDegrees);
  public static final double kHomeDegrees               =   0.0; // Home position
  public static final double kLevelOneClimbDegrees      =   5.0; // Level 1 climb position
  public static final double kLevelTwoClimbDegrees      = -50.0; // Level 2 climb position
  public static final double kPositionToleranceDegrees  =   2.0; // Tolerance for climber positions (in degrees)

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kStallCurrentThreshold   = 25.5;  // Amps - 85% of smart current limit
  public static final double kStallVelocityThreshold  = 1.0;   // Degrees/sec - indicates motor not moving
  public static final double kMaxVelocityDegPerSec    = 100.0; // Maximum velocity for position control
  public static final double kMaxAccelDegPerSec2      = 200.0; // Maximum acceleration for position control
  public static final double kMoveTimeoutSeconds      = 3.0;   // Timeout for move commands
  public static final double kManualUpVoltage         =  12.0; // Magnitude of voltage for manual up control (direction handled via kUpDirection)
  public static final double kManualDownVoltage       =  12.0; // Magnitude of voltage for manual down control (direction handled via kUpDirection)

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // ------------------------------------------------------------
  public static final double kClimberKP = 0.05; // Proportional gain for position control
  public static final double kClimberKI = 0.0; // Integral gain for position control
  public static final double kClimberKD = 0.0; // Derivative gain for position control
}
