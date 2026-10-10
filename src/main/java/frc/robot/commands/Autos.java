package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.Utils;

/**
 * Autos command factory that defines autonomous routines for the robot.
 */
public class Autos {
  private final Feedback feedback;
  private final DriveSubsystem driveSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final ClimberSubsystem climberSubsystem;

  /**
   * Creates a Feedback command factory that can control both the LED and rumble subsystems. 
   * @param driveSubsystem the drive subsystem to control
   * @param ledSubsystem the LED subsystem to control
   * @param visionSubsystem the vision subsystem to control
   * @param climberSubsystem the climber subsystem to control
   */
  public Autos(
    Feedback feedback,
    DriveSubsystem driveSubsystem,
    FeederSubsystem feederSubsystem,
    FlywheelSubsystem flywheelSubsystem,
    TurretSubsystem turretSubsystem,
    ClimberSubsystem climberSubsystem
  ) {
    this.feedback = feedback;
    this.driveSubsystem = driveSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.climberSubsystem = climberSubsystem;
  }

  /**
   * Example autonomous routine that drives forward for 2 seconds, then stops.
   * @return the command representing the autonomous routine
   */
  public Command exampleAutoRoutine() {
    return Commands.parallel(
      driveSubsystem.alignToTagCommand(
        () -> 1, 
        () -> 0.25, 
        () -> 0.25
      ),
      climberSubsystem.toLowerLimitCommand()
    )
    .withTimeout(5)
    .andThen(Commands.parallel(
      Commands.waitUntil(
        flywheelSubsystem.isFlywheelAtTargetTrigger
        .and(turretSubsystem.isAtAngleTrigger)
      ).andThen(feederSubsystem.feedCommand()),
      turretSubsystem.aimAtPoseCommand(
        () -> turretSubsystem.getPose(driveSubsystem.getPose()), 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose,
        () -> driveSubsystem.getFieldRelativeSpeeds()
      ),
      flywheelSubsystem.shootAtPoseCommand(      
        () -> turretSubsystem.getPose(driveSubsystem.getPose()), 
        () -> Utils.isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose
      )
    ))
    .withTimeout(10)
    .andThen(driveSubsystem.driveToPoseCommand(null))
    .andThen(climberSubsystem.toLevelOneCommand())
    .andThen(feedback.successCommand());
  }
}
