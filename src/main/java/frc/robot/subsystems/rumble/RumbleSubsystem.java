// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rumble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.util.Utils;

/**
 * RumbleSubsystem controls the Xbox controller rumble to provide haptic feedback.
 */
public class RumbleSubsystem extends SubsystemBase {
  // Controller reference for rumble
  private CommandXboxController controller;
      
  /**
   * Creates a new FeedbackSubsystem.
   * @param controller The Xbox controller to use for rumble feedback
   */
  public RumbleSubsystem(CommandXboxController controller) {
    // Set controller reference
    this.controller = controller;

    // Set subsystem default command to stop rumble when no other command is running
    setDefaultCommand(stopCommand());
    
    // Output initialization progress
    Utils.logInfo("Rumble subsystem initialized");
  }

  @Override
  public void periodic() {}

  // ==================== Rumble Control Methods ====================
  
  /**
   * Set controller rumble
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  private void setRumble(double intensity) {
    if (controller != null) {
      double clampedIntensity = MathUtil.clamp(intensity, 0.0, 1.0);
      controller.getHID().setRumble(RumbleType.kBothRumble, clampedIntensity);
    }
  }

  /**
   * Stop controller rumble
   */
  private void stop() {
    setRumble(0);
  }
  
  // ==================== Command Factories ====================

  /**
   * Command to rumble controller for a duration and then stop
   * @param intensity Rumble intensity (0.0 to 1.0)
   * @param duration Duration in seconds (0.0 to 10.0)
   * @return Command that rumbles then stops
   */
  public Command rumbleCommand(double intensity, double duration) {
    return run(() -> setRumble(intensity))
      .withTimeout(MathUtil.clamp(duration, 0.0, 10.0))
      .finallyDo(this::stop)
      .withName("Rumble_Custom");
  }
  
  /**
   * Command to stop rumble
   * @return Command that stops the rumble
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Rumble_Stop");
  }
  
  /**
   * Command for short rumble pulse
   * @return Command that does a quick rumble
   */
  public Command quickCommand() {
    return rumbleCommand(0.5, 0.2)
      .withName("Rumble_Quick");
  }
  
  /**
   * Command for medium rumble pulse
   * @return Command that does a medium rumble
   */
  public Command mediumCommand() {
    return rumbleCommand(0.7, 0.4)
      .withName("Rumble_Medium");
  }
  
  /**
   * Command for strong rumble pulse
   * @return Command that does a strong rumble
   */
  public Command strongCommand() {
    return rumbleCommand(1.0, 0.4)
      .withName("Rumble_Strong");
  }
  
  /**
   * Command for double rumble pulse
   * @return Command that rumbles twice
   */
  public Command doubleCommand() {
    return rumbleCommand(0.7, 0.15)
      .andThen(Commands.waitSeconds(0.1))
      .andThen(rumbleCommand(0.7, 0.15))
      .withName("Rumble_Double");
  }
}
