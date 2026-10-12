package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.rumble.RumbleSubsystem;

/**
 * Feedback command factory that provides a simple interface to trigger both
 * LED and rumble feedback together. It can be used to give consistent 
 * feedback for info, success, warning, and error etc.
 */
public class Feedback {
  private final LEDSubsystem ledSubsystem;
  private final RumbleSubsystem rumbleSubsystem;

  /**
   * Creates a Feedback command factory that can control both the LED and rumble subsystems.
   * @param ledSubsystem the LED subsystem to control
   * @param rumbleSubsystem the Rumble subsystem to control
   */
  public Feedback(LEDSubsystem ledSubsystem, RumbleSubsystem rumbleSubsystem) {
    this.ledSubsystem = ledSubsystem;
    this.rumbleSubsystem = rumbleSubsystem;
  }

  /**
   * Success feedback: flashes green briefly and does a double rumble
   * @return Command to run the success feedback routine
   */
  public Command successCommand() {
    return Commands.parallel(
      ledSubsystem.successCommand(),
      rumbleSubsystem.doubleCommand()
    ).withName("Feedback_Success");
  }

  /**
   * Info feedback: flashes cyan briefly and does a quick rumble
   * @return Command to run the info feedback routine
   */
  public Command infoCommand() {
    return Commands.parallel(
      ledSubsystem.infoCommand(),
      rumbleSubsystem.quickCommand()
    ).withName("Feedback_Info");
  }

  /**
   * Warning feedback: flashes gold briefly and does a medium rumble
   * @return Command to run the warning feedback routine
   */
  public Command warningCommand() {
    return Commands.parallel(
      ledSubsystem.warningCommand(),
      rumbleSubsystem.mediumCommand()
    ).withName("Feedback_Warning");
  }

  /**
   * Hardware error feedback: locks magenta on the LED strip and does a strong rumble
   * @return Command to run the error feedback routine
   */
  public Command errorCommand() {
    return Commands.parallel(
      ledSubsystem.errorCommand(),
      rumbleSubsystem.strongCommand()
    ).withName("Feedback_Error");
  }
}
