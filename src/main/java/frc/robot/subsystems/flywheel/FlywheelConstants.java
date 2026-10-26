// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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

  
  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kFlywheelMinRPM          = 0.0;    // Minimum RPM for flywheel
  public static final double kFlywheelMaxRPM          = 6000.0; // Maximum RPM for flywheel
  public static final double kFlywheelToleranceRPM    = 100.0;  // RPM window to consider flywheel at target
  public static final double kFlywheelMinSpinningRPM  = 100.0;  // RPM threshold to consider flywheel spinning

  // Average fuel exit speed used for time-of-flight / shoot-on-the-move lead. TODO: calibrate
  // against measured shots (it should correspond to the RPM the table commands).
  public static final double kShotSpeedMetersPerSecond = 12.0;

  public static final double kFlywheelDefaultDistanceToTarget = 2.0; // Default distance to target if robot or target pose is null
  public static final double kShotLatencySeconds = 0.10; // placeholder: measure/tune

  // Distance (m) -> time of flight (s). Placeholder values: measure with slow-mo video
  public static final InterpolatingDoubleTreeMap kTimeOfFlightSeconds = new InterpolatingDoubleTreeMap();
  static {
    kTimeOfFlightSeconds.put(1.5, 0.45);
    kTimeOfFlightSeconds.put(3.0, 0.65);
    kTimeOfFlightSeconds.put(5.0, 0.90);
  }

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
