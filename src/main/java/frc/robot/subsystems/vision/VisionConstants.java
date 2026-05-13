// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class VisionConstants {
  // Define AprilTag vision cameras and their transforms relative to the robot center
  // The front of the robot is the intake side, the back is the launcher side
  // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  //      for coordinate system conventions
  public static final Map<String, Transform3d> kCameraConfigs = Map.of(
    "VISION_FRONT", new Transform3d(
      new Translation3d(
        Units.inchesToMeters(8),    // forward 8 inches
        Units.inchesToMeters(-6),          // right 6 inches  
        Units.inchesToMeters(12)    // up 12 inches 
      ),
      new Rotation3d(0, 0, 0) 
    ),
    "VISION_BACK", new Transform3d(
      new Translation3d(
        Units.inchesToMeters(8),    // forward 8 inches
        Units.inchesToMeters(6),    // left 6 inches  
        Units.inchesToMeters(12)    // up 12 inches 
      ),
      new Rotation3d(0, 0, 0) 
    )
  );

  // Vision configuration constants
  public static final boolean kEnableVision             = false;
  public static final double kPoseAmbiguityThreshold    = 0.20;
  public static final double kFieldBorderMargin         = 0.50; // meters
  public static final double kZMargin                   = 0.75; // meters

  // Stand 5-6 meters from a tag
  // Check tag area in PhotonVision UI
  // Set threshold to 80% of that value
  // Example: If tag shows 1200 pixels² at 5m:
  public static final double kMinTagAreaPixels          = 1000.0; // Adjust based on testing
  
  // Standard deviation calculation constants
  public static final double kSingleTagBaseXYstdDev     = 0.08; // meters
  public static final double kSingleTagBaseThetaStdDev  = 0.04; // radians (~2.3 degrees)
  public static final double kMultiTagBaseXYstdDev      = 0.02; // meters
  public static final double kMultiTagBaseThetaStdDev   = 0.01; // radians (~0.6 degrees)
  public static final double kMaxDistanceMeters         = 6.00; // anything over this is max std dev
}
