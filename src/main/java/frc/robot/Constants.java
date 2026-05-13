// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Analog IO contants
   */
  public static final class AnalogConstants {
    // Swerve module absolute encoders are defined in /deploy/swerve/YYYY/modules/*.json
    // Ensure that analog constants defined here do not conflict  with file definitions
  }

  /**
   * CAN bus IO contants
   */
  public static final class CANConstants {
    // Swerve module drive & steer motor IDs are defined in /deploy/swerve/YYYY/modules/*.json
    // Ensure that CAN constants defined here do not conflict with file definitions
    // Reserve CAN IDs 0 = Navx, 1-19 for swerve modules

    // Climber motor ID
    public static final int kClimberMotorID = 20;

    // Elevator motor IDs
    public static final int kElevatorLeaderMotorID = 21;
    public static final int kElevatorFollowerMotorID = 22;

    // Turret & flywheel motor IDs
    public static final int kTurretMotorID = 23;
    public static final int kFlywheelLeaderMotorID = 24;
    public static final int kFlywheelFollowerMotorID = 25;
  }
  
  /**
   * Digital IO constants
   */
  public static final class DIOConstants {}

  /**
   * PWM IO constants
   */
  public static class PWMConstants {
    public static final int kLEDStringID = 0;
  }

  /**
   * Field constants
   */
  public static class FieldConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final double kFieldLengthMeters = Units.inchesToMeters(651.22); // meters
    public static final double kFieldWidthMeters = Units.inchesToMeters(317.69); // meters
    public static final Translation2d kBlueHubCenter = new Translation2d(Units.inchesToMeters(158.84), Units.inchesToMeters(182.11));
    public static final Translation2d kRedHubCenter = new Translation2d(Units.inchesToMeters(158.84), Units.inchesToMeters(469.11));
  }
}
