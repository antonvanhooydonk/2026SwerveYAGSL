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
  public static final boolean kEnableVision             = true;
  public static final double kPoseAmbiguityThreshold    = 0.20;
  public static final double kFieldBorderMargin         = 0.50; // meters
  public static final double kZMargin                   = 0.75; // meters
  public static final double kMaxDistanceMeters         = 6.00; // anything over this is max std dev

  // PhotonVision's target area is a PERCENT of the image (0-100), not pixels.
  // Calibrate: stand ~5 m from a tag, read its area in the PhotonVision UI, and set this
  // to ~80% of that value. The 0.08 default is a rough estimate for a 1280x800, ~70 deg HFOV camera
  // (a tag at 5 m is ~0.09%) and MUST be checked against your actual cameras.
  public static final double kMinTagAreaPercent         = 0.08;
  
  // Theta trust. Single-tag heading is ambiguous, so vision heading is ignored
  // and the NavX owns heading. Use a large finite value rather than infinity.
  public static final double kIgnoredThetaStdDev        = 1.0e6;
  public static final double kMultiTagBaseThetaStdDev   = 0.10;  // radians (~5.7 deg), was 0.01
  public static final double kMinThetaStdDev            = 0.05;  // radians (~2.9 deg) floor

  // Single-tag estimates beyond this are rejected outright
  public static final double kSingleTagMaxDistanceMeters = 4.0;

  // XY trust. Single-tag XY is less reliable than multi-tag, so
  // we use a larger base std dev for single-tag estimates.
  public static final double kSingleTagBaseXYstdDev     = 0.08; // meters
  public static final double kMultiTagBaseXYstdDev      = 0.02; // meters

  // ----------------------------------------------------------
  // Define the robot's cameras
  // ----------------------------------------------------------
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
        Units.inchesToMeters(-8),          // forward 8 inches
        Units.inchesToMeters(6),    // left 6 inches  
        Units.inchesToMeters(12)    // up 12 inches 
      ),
      new Rotation3d(0, 0, Math.PI) 
    )
  );
}
