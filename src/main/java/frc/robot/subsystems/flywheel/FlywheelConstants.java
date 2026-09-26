// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class FlywheelConstants {  
  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------
  public static final boolean kFlywheelFollowerOpposesLeader = true; // True if motors are mechanically mirrored

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kFlywheelMinRPM = 0.0;    // Minimum RPM for flywheel
  public static final double kFlywheelMaxRPM = 6000.0; // Maximum RPM for flywheel
  public static final double kFlywheelToleranceRPM    = 50.0;  // RPM window to consider flywheel at target
  public static final double kFlywheelMinSpinningRPM  = 100.0; // RPM threshold to consider flywheel spinning

  public static final double kFlywheelDefaultDistanceToTarget = 2.0; // Default distance to target if robot or target pose is null
  
  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // Tuning: start with kV only (kP = 0), add kP if error remains
  // ------------------------------------------------------------
  public static final double kFlywheelKP = 0.0;
  public static final double kFlywheelKI = 0.0;
  public static final double kFlywheelKD = 0.0;
  public static final double kFlywheelKS = 0.0; // Static friction - from SysId
  public static final double kFlywheelKV = 0.0; // Velocity feedforward - from SysId
  public static final double kFlywheelKA = 0.0; // Acceleration feedforward - from SysId
}
