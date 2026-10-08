package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.Utils;

/**
 * Scoring command factory that provides a simple interface to trigger both
 * turret and shooter actions together. It can be used to give consistent 
 * behavior for scoring at the alliance hub.
 */
public class Scoring {
  private final DriveSubsystem driveSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;

  /**
   * Creates a scoring command factory that can coordinate the turret and shooter subsystems.
   * @param driveSubsystem the drive subsystem to control
   * @param feederSubsystem the feeder subsystem to control
   * @param flywheelSubsystem the flywheel subsystem to control
   * @param turretSubsystem the turret subsystem to control
   */
  public Scoring(
    DriveSubsystem driveSubsystem, 
    FeederSubsystem feederSubsystem, 
    FlywheelSubsystem flywheelSubsystem,
    TurretSubsystem turretSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  /**
   * Score at the current alliance hub by auto-aiming and and auto-adjusting shooting
   * power based on the distance to the hub.
   * @return Command to auto-score at the current alliance hub
   */
  public Command scoreCommand() {
    return Commands.parallel(
      Commands.waitUntil(
        flywheelSubsystem.isFlywheelAtTargetTrigger
        .and(turretSubsystem.isAtAngleTrigger)
      ).andThen(feederSubsystem.feedCommand()),
      turretSubsystem.aimAtPoseCommand(
        driveSubsystem::getTurretPose, 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose,
        () -> driveSubsystem.getFieldRelativeSpeeds()
      ),
      flywheelSubsystem.shootAtPoseCommand(
        driveSubsystem::getTurretPose, 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose,
        driveSubsystem::getFieldRelativeSpeeds
      )
    );
  }

  /**
   * Pass fuel to the current alliance side by auto-aiming and and auto-adjusting shooting
   * power based on the distance to the "catcher" poses.
   * @return Command to auto-pass to the current alliance side
   */
  public Command passCommand() {
    return Commands.parallel(
      Commands.waitUntil(
        flywheelSubsystem.isFlywheelAtTargetTrigger
        .and(turretSubsystem.isAtAngleTrigger)
      ).andThen(feederSubsystem.feedCommand()),
      turretSubsystem.aimAtPoseCommand(
        driveSubsystem::getTurretPose, 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedPassPose : FieldConstants.kBluePassPose,
        () -> driveSubsystem.getFieldRelativeSpeeds()
      ),
      flywheelSubsystem.shootAtPoseCommand(
        driveSubsystem::getTurretPose, 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedPassPose : FieldConstants.kBluePassPose,
        driveSubsystem::getFieldRelativeSpeeds
      )
    );
  }
}
