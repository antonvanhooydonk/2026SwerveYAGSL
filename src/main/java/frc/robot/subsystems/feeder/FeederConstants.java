// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class FeederConstants {  
  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------

  
  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kFeederMinRPM          = -6000.0; // Minimum RPM for feeder
  public static final double kFeederMaxRPM          =  6000.0; // Maximum RPM for feeder
  public static final double kFeederToleranceRPM    =    50.0; // RPM window to consider feeder at target
  public static final double kFeederMinSpinningRPM  =   100.0; // RPM threshold to consider feeder spinning
  public static final double kFeedRPM               =  3000.0; // RPM for feeding fuel
  public static final double kReverseRPM            = -3000.0; // RPM for reversing feeder

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // Tuning: start with kV only (kP = 0), add kP if error remains
  // ------------------------------------------------------------
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0; // Static friction - from SysId
  public static final double kV = 0.0; // Velocity feedforward - from SysId
  public static final double kA = 0.0; // Acceleration feedforward - from SysId
}
