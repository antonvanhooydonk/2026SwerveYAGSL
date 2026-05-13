package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Autos command factory that defines autonomous routines for the robot.
 */
public class Autos {
  private final SwerveSubsystem driveSubsystem;
  private final LEDSubsystem ledSubsystem;
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
    SwerveSubsystem driveSubsystem, 
    LEDSubsystem ledSubsystem,
    VisionSubsystem visionSubsystem,
    ClimberSubsystem climberSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.climberSubsystem = climberSubsystem;
  }

  /**
   * Example autonomous routine that drives forward for 2 seconds, then stops.
   * @return the command representing the autonomous routine
   */
  public Command exampleAutoRoutine() {
    return driveSubsystem.alignToTagCommand(1, 0, 0).withTimeout(2)
      .andThen(ledSubsystem.successCommand());
  }
}
