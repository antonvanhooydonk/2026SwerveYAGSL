// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // Navx 3 ID
    // SPI = 0

    // Reserved for swerve modules
    // FL: encoder = A0, steer = CAN2, drive = CAN11
    // FR: encoder = A1, steer = CAN8, drive = CAN12
    // BL: encoder = A3, steer = CAN4, drive = CAN14
    // BR: encoder = A2, steer = CAN6, drive = CAN13

    // Arm motor ID
    public static final int kArmMotorID = 15;

    // Climber motor ID
    public static final int kClimberMotorID = 16;

    // Elevator motor IDs
    public static final int kElevatorLeaderMotorID = 17;
    public static final int kElevatorFollowerMotorID = 18;

    // Intake motor & solenoid IDs
    public static final int kIntakeRollerMotorID = 19;
    public static final int kIntakeSolenoidMotorID = 20;

    // Turret & flywheel motor IDs
    public static final int kTurretMotorID = 21;
    public static final int kFlywheelLeaderMotorID = 22;
    public static final int kFlywheelFollowerMotorID = 23;
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
    
    // Field frame: origin at the blue alliance corner, +X toward the red alliance wall, +Y across the 
    // field width. The hubs sit 182.11 in from their own alliance wall on the field centerline (Y = width / 2).
    // Red is derived by mirroring X so the two can never drift out of sync.
    // VERIFY against the hub AprilTag poses in kFieldLayout before your first event.
    public static final Translation2d kBlueHubCenter = new Translation2d(Units.inchesToMeters(182.11), kFieldWidthMeters / 2.0);
    public static final Translation2d kRedHubCenter = new Translation2d(kFieldLengthMeters - kBlueHubCenter.getX(), kFieldWidthMeters / 2.0);
    public static final Pose2d kBlueHubPose = new Pose2d(kBlueHubCenter, new Rotation2d());
    public static final Pose2d kRedHubPose = new Pose2d(kRedHubCenter, new Rotation2d(-180.0));

    // TODO: placeholders - these are currently the hub centers. Replace with the real pass targets.
    public static final Pose2d kBluePassPose = new Pose2d(kBlueHubCenter, new Rotation2d());
    public static final Pose2d kRedPassPose = new Pose2d(kRedHubCenter, new Rotation2d(-180.0));
  }
}
