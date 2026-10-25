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
  // Vision configuration constants
  public static final boolean kEnableVision             = true; // ensure true at competitions
  public static final boolean kDebug                    = true && kEnableVision; // ensure false at competitions
  public static final double kPoseAmbiguityThreshold    = 0.20;
  public static final double kFieldBorderMargin         = 0.50; // meters
  public static final double kZMargin                   = 0.75; // meters

  // PhotonVision's target area is a PERCENT of the image (0-100), not pixels.
  // Calibrate: stand ~5 m from a tag, read its area in the PhotonVision UI, and set this
  // to ~80% of that value. The 0.08 default is a rough estimate for a 1280x800, ~70 deg HFOV camera
  // (a tag at 5 m is ~0.09%) and MUST be checked against your actual cameras.
  public static final double kMinTagAreaPercent         = 0.08;
  
  // Vision std dev model: stdDev = baseline * distance^2 / tagCount
  public static final double kLinearStdDevBaseline        = 0.02;  // meters at 1 m with one tag
  public static final double kAngularStdDevBaseline       = 0.06;  // radians at 1 m with one tag
  public static final double kMinXYStdDev                 = 0.02;  // meters, floor (never fully trust vision)
  public static final double kMaxXYStdDev                 = 1.0;   // meters, ceiling
  public static final double kMinThetaStdDev              = 0.05;  // radians, floor
  public static final double kIgnoredThetaStdDev          = 1.0e6; // effectively "do not use heading"
  public static final double kMaxDistanceMeters           = 6.00;  // multi-tag estimates beyond this are rejected
  public static final double kSingleTagMaxDistanceMeters  = 4.00;  // single-tag estimates beyond this are rejected
  
  // ----------------------------------------------------------
  // Define the robot's cameras
  // IMPORTANT:
  //   - Keep camera types separate from each other (AprilTag, ObjectDetection, DriverFeed)
  //   - The vision susbsystem currently only supports AprilTag cameras
  // ----------------------------------------------------------
  // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  //      for coordinate system conventions
  public static final Map<String, Transform3d> kAprilTagCameras = Map.of(
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
        Units.inchesToMeters(-8),          // forward 8 inches
        Units.inchesToMeters(6),    // left 6 inches  
        Units.inchesToMeters(12)    // up 12 inches 
      ),
      new Rotation3d(0, 0, Math.PI) 
    )
  );

  public static final Map<String, Transform3d> kObjectDetectionCameras = Map.of();
  public static final Map<String, Transform3d> kDriverFeedCameras = Map.of();
}
