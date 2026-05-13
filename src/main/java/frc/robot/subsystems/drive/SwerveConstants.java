// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants {
  // ------------------------------------------------------------
  // Maximum drive & turning speeds - adjust as necessary
  // ------------------------------------------------------------
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14); // max translational speed, ensure <= kMaxDriveVelocityAt12VoltsMPS (15.5), set 7 in loft, 10 - 14 at competition
  public static final double kMaxAngularSpeedRadsPerSecond = Units.rotationsToRadians(1.75); // max rotational speed, conservative: 1.0 - 1.5, competitive: 1.5 - 2.0, aggressive: 2.0 - 2.5
  public static final double kSlowModeScaling = 0.25; // scaling factor for "slow mode", typically 0.25 to 0.5, adjust as needed for driver control at low speeds

  public static final boolean kUseSetpointGenerator = true && Robot.isReal();

  // ------------------------------------------------------------
  // Driver joystick settings
  // ------------------------------------------------------------
  public static final double kJoystickSmoothing = 2.0; // 2 to square, 3 to cube
  public static final double kJoystickDeadband  = 0.1; // typically 0.05 to 0.15

  // ------------------------------------------------------------
  // Vision fusion measurement settings
  // ------------------------------------------------------------  
  /**
   * Maximum age of vision measurements to accept (seconds)
   */
  public static final double kVisionMeasurementMaxAge = 0.3;
  
  /**
   * Maximum allowed translation jump for added vision measurements (meters)
   * Rejects vision measurements that are too far from current estimate
   */
  public static final double kVisionMaxTranslationJumpMeters = 1.0;

  // ============================================================
  // BELOW THIS LINE SHOULDN'T BE CHANGED AT COMPETITION
  // ============================================================

  // ------------------------------------------------------------
  // Physical constants - adjust these to robot each year
  // ------------------------------------------------------------
  public static final double kTrackWidthMeters = Units.inchesToMeters(23); // distance between left and right wheels
  public static final double kWheelBaseMeters = Units.inchesToMeters(23); // distance between front and back wheels
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
  public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
  public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
  public static final double kDriveGearRatio = 6.75; // Drive gear ratio for MKi L2
  public static final double kSteerGearRatio = 21.4285714286; // Steering gear ratio
  public static final double kWheelCOF = 1.19; // could try 1.0 to 1.3, coefficient of friction of wheel on carpet
  public static final double kRobotMassKg = Units.lbsToKilograms(134);
  public static final double kRobotMOI = 1/12 * kRobotMassKg * ((kTrackWidthMeters * kTrackWidthMeters) + (kWheelBaseMeters * kWheelBaseMeters)); // kg m^2, moment of inertia about center of robot
  public static final double kMaxDriveVelocityAt12VoltsMPS = Units.feetToMeters(15.5); // MK4i L2 Kraken non-FOC With 14t pinion (https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033798445)
   
  public static final double kGyroAngleOffsetDegrees = 0.0; // default 0.0 - Rotate the gyro X axis if the gyro was not installed facing forward
  public static final double kPeriodicTimeSeconds = 0.02; // 20ms (default)

  // Define the module translations from the robot's center
  // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#coordinate-system
  public static final Translation2d[] kSwerveModuleTranslations = new Translation2d[] {
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(11.5)), // FL
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(-11.5)),       // FR
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(11.5)),       // BL
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-11.5))              // BR
  };

  // ------------------------------------------------------------
  // Autobuilder Constants
  // ------------------------------------------------------------
  public static final PathConstraints kPathfindingConstraints = new PathConstraints(
    3.0, // Velocity (m/s)
    4.0, // Acceleration (m/s^2)
    Units.degreesToRadians(540), // Angular velocity (rad/s)
    Units.degreesToRadians(720) // Angular acceleration (rad/s^2)
  );

  public static final RobotConfig kRobotConfig = new RobotConfig(
    kRobotMassKg,
    kRobotMOI,
    new ModuleConfig(
      kWheelRadiusMeters,
      kMaxDriveVelocityAt12VoltsMPS,
      kWheelCOF, 
      DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio),
      60.0,
      1
    ),
    kSwerveModuleTranslations
  );
}
