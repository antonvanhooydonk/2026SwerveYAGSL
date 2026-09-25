package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Autos command factory that defines autonomous routines for the robot.
 */
public class Autos {
  private final Feedback feedback;
  private final SwerveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final VisionSubsystem visionSubsystem;
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
    SwerveSubsystem driveSubsystem,
    ShooterSubsystem shooterSubsystem,
    TurretSubsystem turretSubsystem,
    VisionSubsystem visionSubsystem,
    ClimberSubsystem climberSubsystem
  ) {
    this.feedback = feedback;
    this.driveSubsystem = driveSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.climberSubsystem = climberSubsystem;
  }

  /**
   * Example autonomous routine that drives forward for 2 seconds, then stops.
   * @return the command representing the autonomous routine
   */
  public Command exampleAutoRoutine() {
    return driveSubsystem.alignToTagCommand(
      () -> 1, 
      () -> 0.25, 
      () -> 0.25
    )
    .withTimeout(15)
    .andThen(turretSubsystem.aimAtPoseCommand(null, null))
    .andThen(shooterSubsystem.shootAtPoseCommand(null, null))
    .andThen(feedback.successCommand());
  }
}
