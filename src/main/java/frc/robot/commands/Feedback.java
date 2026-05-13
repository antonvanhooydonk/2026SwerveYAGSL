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
   * Give success feedback: flashes green for 1 second and does a double rumble
   * @return Command to run the success feedback routine
   */
  public Command successCommand() {
    return successCommand(1.0);
  }

  /**
   * Give success feedback: flashes green for a specified duration and does a double rumble
   * @param timeoutSeconds Duration in seconds for the LED feedback
   * @return Command to run the success feedback routine
   */
  public Command successCommand(double timeoutSeconds) {
    return Commands.parallel(
      ledSubsystem.successCommand().withTimeout(timeoutSeconds),
      rumbleSubsystem.doubleCommand()
    ).withName("Feedback_Success");
  }

  /**
   * Give info feedback: flashes blue for 1 second and does a quick rumble
   * @return Command to run the info feedback routine
   */
  public Command infoCommand() {
    return infoCommand(1.0);
  }

  /**
   * Give info feedback: flashes blue for a specified duration and does a quick rumble
   * @param timeoutSeconds Duration in seconds for the LED feedback
   * @return Command to run the info feedback routine
   */
  public Command infoCommand(double timeoutSeconds) {
    return Commands.parallel(
      ledSubsystem.infoCommand().withTimeout(timeoutSeconds),
      rumbleSubsystem.quickCommand()
    ).withName("Feedback_Info");
  }

  /**
   * Give warning feedback: flashes yellow for 1 second and does a medium rumble
   * @return Command to run the warning feedback routine
   */
  public Command warningCommand() {
    return warningCommand(1.0);
  }

  /**
   * Give warning feedback: flashes yellow for a specified duration and does a medium rumble
   * @param timeoutSeconds Duration in seconds for the LED feedback
   * @return Command to run the warning feedback routine
   */
  public Command warningCommand(double timeoutSeconds) {
    return Commands.parallel(
      ledSubsystem.warningCommand().withTimeout(timeoutSeconds),
      rumbleSubsystem.mediumCommand()
    ).withName("Feedback_Warning");
  }

  /**
   * Give error feedback: flashes red for 1 second and does a strong rumble
   * @return Command to run the error feedback routine
   */
  public Command errorCommand() {
    return errorCommand(1.0);
  }

  /**
   * Give error feedback: flashes red for a specified duration and does a strong rumble
   * @param timeoutSeconds Duration in seconds for the LED feedback
   * @return Command to run the error feedback routine
   */
  public Command errorCommand(double timeoutSeconds) {
    return Commands.parallel(
      ledSubsystem.errorCommand().withTimeout(timeoutSeconds),
      rumbleSubsystem.strongCommand()
    ).withName("Feedback_Error");
  }
}
