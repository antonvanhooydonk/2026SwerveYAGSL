package frc.robot.util;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.drive.DriveConstants;

/**
 * Guides the drive team in placing the robot at the starting pose of the selected auto.
 * Call update() repeatedly from disabledPeriodic(). Requires a meaningful pose 
 * estimate (e.g. seeded by AprilTag vision).
 */
public class StartingPoseGuide {
  private final Supplier<String> autoNameSupplier;
  private final Supplier<Pose2d> poseSupplier;
  private final Field2d field = new Field2d();

  private String lastAutoName = null;
  private Pose2d cachedStartingPose = null;

  /**
   * @param autoNameSupplier Supplies the selected auto name (e.g. autoChooser::getSelected)
   * @param poseSupplier Supplies the robot's current pose (e.g. driveSubsystem::getPose)
   */
  public StartingPoseGuide(Supplier<String> autoNameSupplier, Supplier<Pose2d> poseSupplier) {
    this.autoNameSupplier = autoNameSupplier;
    this.poseSupplier = poseSupplier;
    SmartDashboard.putData("Auto Align/Field", field);
  }

  public void update() {
    String autoName = autoNameSupplier.get();
    if (autoName == null || autoName.isEmpty()) {
      SmartDashboard.putString("Auto Align/Status", "No auto selected");
      return;
    }

    // Load & cache the starting pose only when the selection changes
    if (!autoName.equals(lastAutoName)) {
      lastAutoName = autoName;
      try {
        // Starting pose is always relative to a blue alliance origin
        cachedStartingPose = new PathPlannerAuto(autoName).getStartingPose();
        SmartDashboard.putString("Auto Align/Status", "Ready");
      } catch (Exception ex) {
        cachedStartingPose = null;
        SmartDashboard.putString("Auto Align/Status", "Couldn't load starting pose for \"" + autoName + "\"");
        return;
      }
    }
    if (cachedStartingPose == null) {
      return;
    }

    Pose2d targetPose = Utils.isRedAlliance()
      ? FlippingUtil.flipFieldPose(cachedStartingPose)
      : cachedStartingPose;
    Pose2d currentPose = poseSupplier.get();

    // Error in the ROBOT's frame: +X = move forward, +Y = move left, +rotation = rotate CCW
    Transform2d error = targetPose.minus(currentPose);
    double forwardMeters = error.getX();
    double leftMeters = error.getY();
    double rotateDegrees = error.getRotation().getDegrees();
    double totalErrorMeters = error.getTranslation().getNorm();

    SmartDashboard.putString("Auto Align/Selected Auto", autoName);
    SmartDashboard.putString("Auto Align/Move", String.format(
      "%s %.2fm, %s %.2fm, %s %.1f°",
      forwardMeters >= 0 ? "Fwd" : "Back", Math.abs(forwardMeters),
      leftMeters >= 0 ? "Left" : "Right", Math.abs(leftMeters),
      rotateDegrees >= 0 ? "CCW" : "CW", Math.abs(rotateDegrees)));
    SmartDashboard.putBoolean("Auto Align/In Position",
      totalErrorMeters < DriveConstants.kStartPoseTranslationToleranceMeters
      && Math.abs(rotateDegrees) < DriveConstants.kStartPoseRotationToleranceDegrees);

    // Visual "you are here" (robot) vs "target" on its own field widget
    field.setRobotPose(currentPose);
    field.getObject("Target").setPose(targetPose);
  }
}
