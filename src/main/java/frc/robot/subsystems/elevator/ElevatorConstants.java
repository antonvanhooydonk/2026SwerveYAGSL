// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/**
 * Constants for the elevator subsystem.
 */
public final class ElevatorConstants {
  // ------------------------------------------------------------
  // Physical constants - adjust for your robot
  // ------------------------------------------------------------
  public static final double kGearRatio = 10.0; // Motor rotations per output shaft rotation
  public static final double kSpoolDiameterMeters = Units.inchesToMeters(1.5); // Diameter of the spool/sprocket
  public static final double kSpoolCircumferenceMeters = kSpoolDiameterMeters * Math.PI;
  public static final boolean kFollowerOpposesLeader = true; // True if follower is mechanically mirrored

  // ------------------------------------------------------------
  // Height limits - adjust for your robot
  // ------------------------------------------------------------
  public static final double kMinHeightMeters = 0.0; // Bottom of travel (home position)
  public static final double kMaxHeightMeters = Units.inchesToMeters(50.0); // Top of travel

  // ------------------------------------------------------------
  // Preset heights - adjust for your robot's scoring positions
  // ------------------------------------------------------------
  public static final double kHeightStowMeters   = Units.inchesToMeters(0.0);
  public static final double kHeightL1Meters     = Units.inchesToMeters(12.0);
  public static final double kHeightL2Meters     = Units.inchesToMeters(24.0);
  public static final double kHeightL3Meters     = Units.inchesToMeters(36.0);
  public static final double kHeightL4Meters     = Units.inchesToMeters(50.0);

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kHeightToleranceMeters = Units.inchesToMeters(0.5); // Tolerance for isAtTarget

  // ------------------------------------------------------------
  // Homing constants
  // Without limit switches, we drive slowly down until stalled
  // ------------------------------------------------------------
  public static final double kHomingVoltage              = -1.0; // Volts - slow downward voltage
  public static final double kHomingVelocityThresholdMPS = 0.01; // m/s - velocity threshold to detect hard stop

  // ------------------------------------------------------------
  // MotionMagic constraints
  // Tuning: start low, increase until motion is fast but smooth
  // ------------------------------------------------------------
  public static final double kCruiseVelocityMPS = 1.0;  // meters per second
  public static final double kAccelerationMPS2  = 2.0;  // meters per second squared
  public static final double kJerkMPS3          = 20.0; // meters per second cubed (0 to disable)

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // ------------------------------------------------------------
  public static final double kP = 1.0;  // Start low, increase until responsive without overshoot
  public static final double kI = 0.0;  // Avoid unless steady-state error
  public static final double kD = 0.0;  // Add small value if oscillating
  public static final double kS = 0.0;  // Static friction - from SysId
  public static final double kV = 0.0;  // Velocity feedforward - from SysId
  public static final double kA = 0.0;  // Acceleration feedforward - from SysId
  public static final double kG = 0.0;  // Gravity compensation - tune until elevator holds position
}
