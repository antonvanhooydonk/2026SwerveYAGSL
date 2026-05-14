// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PWMConstants;
import frc.robot.util.Utils;

/**
 * LED subsystem for controlling an addressable LED strip and providing visual feedback. 
 */
public class LEDSubsystem extends SubsystemBase {
  // LED hardware
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
      
  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    // Initialize LED strip
    ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    ledStrip = new AddressableLED(PWMConstants.kLEDStringID);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Set subsystem default command to turn off LEDs when no other command is running
    setDefaultCommand(offCommand());
    
    // Output initialization progress
    Utils.logInfo("LED subsystem initialized");
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }

  // ==================== LED Control Methods ====================

  /**
   * Creates a command that runs a pattern on the entire LED strip until interrupted. 
   * The pattern will be applied to the entire strip, and will continue to run until 
   * the command is interrupted or replaced by another command.
   * @param pattern the LED pattern to run
   */
  private Command runPattern(LEDPattern pattern) {
    return run(() -> pattern
      .atBrightness(Percent.of(LEDConstants.kBrightness * 100))
      .applyTo(ledBuffer))
    .ignoringDisable(true);
  }

  // ==================== Command Factories ====================
  
  /**
   * Command to set the LED strip off until another command takes over
   * @return Command that sets LED display off
   */
  public Command offCommand() {
    return runPattern(LEDPattern.solid(Color.kBlack))
      .withName("LED_Off");
  }
  
  /**
   * Command to indicate success state until interrupted
   * @return Command with success LED
   */
  public Command successCommand() {
    return runPattern(LEDPattern.solid(Color.kGreen).breathe(Seconds.of(0.5)))
      .withName("LED_Success");
  }
  
  /**
   * Command to indicate info state until interrupted
   * @return Command with info LED
   */
  public Command infoCommand() {
    return runPattern(LEDPattern.solid(Color.kDarkViolet).breathe(Seconds.of(0.5)))
      .withName("LED_Info");
  }
  
  /**
   * Command to indicate warning state until interrupted
   * @return Command with warning LED
   */
  public Command warningCommand() {
    return runPattern(LEDPattern.solid(Color.kYellow).breathe(Seconds.of(0.5)))
      .withName("LED_Warning");
  }
  
  /**
   * Command to indicate error state until interrupted
   * @return Command with error LED
   */
  public Command errorCommand() {
    return runPattern(LEDPattern.solid(Color.kRed).breathe(Seconds.of(0.5)))
      .withName("LED_Error");
  }
  
  /**
   * Command to set idle feedback until interrupted
   * @return Command that sets idle LED display
   */
  public Command idleCommand() {
    return runPattern(LEDPattern.solid(Color.kSlateBlue).breathe(Seconds.of(2.0)))
      .withName("LED_Idle");
  }
  
  /**
   * Command to display team colors (scrolling gradient) until interrupted  
   * @return Command that scrolls between green & copper
   */
  public Command teamColorsCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kContinuous, 
        Color.kDarkGreen, 
        Color.kDarkOrange
      )
      .scrollAtRelativeSpeed(Percent.per(Second).of(25))
    )
    .withName("LED_TeamColors");
  }

  /**
   * Command to display candy cane pattern (scrolling stripes) until interrupted
   * @return Command that shows rotating red and white stripes
   */
  public Command candyCaneCommand() {
    return runPattern(
      LEDPattern
      .steps(Map.of(0.0, Color.kRed, 0.5, Color.kWhite))
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(1.0 / 60.0))
    )
    .withName("LED_CandyCane");
  }

  /**
   * Command to display disco pattern (scrolling rainbow) until interrupted
   * @return Command that shows random flashing rainbow colors
   */
  public Command discoCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kContinuous,
        Color.kMagenta, 
        Color.kYellow, 
        Color.kCyan, 
        Color.kHotPink,
        Color.kLime, 
        Color.kOrange, 
        Color.kBlue, 
        Color.kMagenta
      )
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(3.5), Meters.of(1.0 / 60.0))
      .blink(Seconds.of(0.07))
    )
    .withName("LED_Disco");
  }

  /**
   * Command to display a fire/flame effect until interrupted.
   * Simulates flames using a warm gradient scrolling upward rapidly.
   * @return Command that shows an animated fire effect
   */
  public Command flameCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kContinuous,
        Color.kBlack, 
        Color.kDarkRed, 
        Color.kOrangeRed, 
        Color.kOrange, 
        Color.kYellow, 
        Color.kBlack
      )
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.2), Meters.of(1.0 / 60.0))
      .breathe(Seconds.of(0.4))
    )
    .withName("LED_Flame");
  }

  /**
   * Command to display a heartbeat pulse effect until interrupted.
   * Double-pulses in red like a cardiac rhythm.
   * @return Command that shows a heartbeat pulse
   */
  public Command heartbeatCommand() {
    return runPattern(
      LEDPattern
      .solid(Color.kRed)
      .breathe(Seconds.of(0.3))
      .overlayOn(LEDPattern.solid(Color.kDarkRed))
    )
    .withName("LED_Heartbeat");
  }

  /**
   * Command to display a meteor/comet trail effect until interrupted.
   * A bright white head fades to a cool blue tail, shooting across the strip.
   * @return Command that shows a streaking meteor
   */
  public Command meteorCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kDiscontinuous,
        Color.kWhite, 
        Color.kCyan, 
        Color.kBlue, 
        Color.kBlack
      )
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.5), Meters.of(1.0 / 60.0))
    )
    .withName("LED_Meteor");
  }

  /**
   * Command to display a Knight Rider / KITT scanner effect until interrupted.
   * A bright red "eye" bounces back and forth across the strip.
   * @return Command that shows a scanning red light
   */
  public Command knightRiderCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kDiscontinuous,
        Color.kBlack, 
        Color.kDarkRed, 
        Color.kRed, 
        Color.kDarkRed, 
        Color.kBlack
      )
      .scrollAtAbsoluteSpeed(MetersPerSecond.of(1.0), Meters.of(1.0 / 60.0))
    )
    .withName("LED_KnightRider");
  }

  /**
   * Command to display a lava lamp / slow plasma effect until interrupted.
   * Deep purples and reds blending and drifting slowly.
   * @return Command that shows a hypnotic lava lamp effect
   */
  public Command lavaLampCommand() {
    return runPattern(
      LEDPattern.gradient(
        GradientType.kContinuous,
        Color.kPurple, 
        Color.kDarkRed, 
        Color.kDeepPink, 
        Color.kDarkMagenta, 
        Color.kPurple
      )
      .scrollAtRelativeSpeed(Percent.per(Second).of(8))
      .breathe(Seconds.of(2.5))
    )
    .withName("LED_LavaLamp");
  }
}
