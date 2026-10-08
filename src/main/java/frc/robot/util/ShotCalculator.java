// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.subsystems.flywheel.FlywheelConstants;

/**
 * Computes the shoot-on-the-move solution in ONE place so the turret (aim) and the flywheel (RPM)
 * always agree: the turret aims at the virtual target and the flywheel is spun for the distance
 * to that same virtual target.
 */
public final class ShotCalculator {
  // Prevent instantiation
  private ShotCalculator() {}

  /**
   * @param virtualTarget field-relative point to aim at (the real target shifted against the robot's motion)
   * @param distanceMeters distance from the shooter to the virtual target (use for the RPM lookup)
   */
  public record Solution(Translation2d virtualTarget, double distanceMeters) {}

  /**
   * @param shooterPose field-relative pose of the shooter (use DriveSubsystem.getTurretPose())
   * @param target field-relative target
   * @param fieldSpeeds field-relative chassis speeds (null is treated as stationary)
   */
  public static Solution solve(Pose2d shooterPose, Translation2d target, ChassisSpeeds fieldSpeeds) {
    Translation2d shooter = shooterPose.getTranslation();
    double vx = fieldSpeeds == null ? 0.0 : fieldSpeeds.vxMetersPerSecond;
    double vy = fieldSpeeds == null ? 0.0 : fieldSpeeds.vyMetersPerSecond;

    Translation2d virtual = target;
    double distance = shooter.getDistance(target);

    // Time of flight depends on the (virtual) distance, which depends on time of flight. Iterate.
    for (int i = 0; i < 3; i++) {
      double timeOfFlight = distance / FlywheelConstants.kShotSpeedMetersPerSecond;
      virtual = target.minus(new Translation2d(vx * timeOfFlight, vy * timeOfFlight));
      distance = shooter.getDistance(virtual);
    }
    return new Solution(virtual, distance);
  }
}
