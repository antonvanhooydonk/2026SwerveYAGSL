// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
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

  private char matchPhase = '?'; // 'A' = autonomous, 'T' = teleop, 'P' = post match
      
  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    // Initialize LED strip
    ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
    ledStrip = new AddressableLED(PWMConstants.kLEDStringID);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Set subsystem default command to turn off LEDs when no other command is running
    // This may be overridden by the match phase initialization methods.
    setDefaultCommand(matchPhaseCommand());
    
    // Output initialization progress
    Utils.logInfo("LED subsystem initialized");
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Applies the given LED pattern to the LED buffer.
   * @param pattern the LED pattern to apply
   */
  private void displayPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
  }

  /**
   * Cancels the currently running command on the LED subsystem, if any.
   * The default command will take over after cancellation.
   */
  private void cancelCurrentCommand() {
    Command current = getCurrentCommand();
    if (current != null) {
      current.cancel();
    }
  }

  /** 
   * Deterministic pseudo-random value in [0, 1] from two integers.
   * @param a first integer
   * @param b second integer
   * @return pseudo-random value in [0, 1]
   */
  private double hash01(int a, int b) {
    int h = a * 374761393 + b * 668265263;
    h = (h ^ (h >>> 13)) * 1274126177;
    h ^= (h >>> 16);
    return (h & 0x7FFFFFFF) / (double) 0x7FFFFFFF;
  }
  
  // ----------------------------------------------------------------------------------------
  // Predefined LED patterns
  // ----------------------------------------------------------------------------------------

  /**
   * Returns a pattern that turns off all LEDs (black).
   * @return LEDPattern that turns off all LEDs
   */
  private LEDPattern offPattern() {
    return LEDPattern.solid(Color.kBlack);
  }

  /**
   * Returns a pattern that indicates success (green) with a breathing effect.
   * @return LEDPattern that indicates success
   */
  private LEDPattern successPattern() {
    return LEDPattern.solid(Color.kGreen);
  }

  /**
   * Returns a pattern that indicates information (dark violet) with a breathing effect.
   * @return LEDPattern that indicates information
   */
  private LEDPattern infoPattern() {
    return LEDPattern.solid(Color.kDarkViolet);
  }

  /**
   * Returns a pattern that indicates a warning (yellow) with a breathing effect.
   * @return LEDPattern that indicates a warning
   */
  private LEDPattern warningPattern() {
    return LEDPattern.solid(Color.kYellow);
  }

  /**
   * Returns a pattern that indicates an error (red) with a breathing effect.
   * @return LEDPattern that indicates an error
   */
  private LEDPattern errorPattern() {
    return LEDPattern.solid(Color.kRed);
  }

  /**
   * Returns a pattern that indicates idle state (slate blue) with a slow breathing effect.
   * @return LEDPattern that indicates idle state
   */
  private LEDPattern idlePattern() {
    return LEDPattern.solid(Color.kSlateBlue).breathe(Seconds.of(2.0));
  }

  /**
   * Returns a solid colour pattern based on the alliance color (red or blue).
   * @return LEDPattern that shows the alliance color
   */
  private LEDPattern alliancePattern() {
    return Utils.isRedAlliance()
      ? LEDPattern.solid(Color.kDarkRed)
      : LEDPattern.solid(Color.kDarkBlue);
  }

  /**
   * Returns a pattern that scrolls between the team colors (dark green and dark orange) 
   * with a continuous gradient.
   * @return LEDPattern that scrolls between the team colors
   */
  private LEDPattern teamColorsPattern() {
    return LEDPattern.gradient(
      GradientType.kContinuous, 
      Color.kDarkGreen, 
      Color.kDarkOrange
    )
    .scrollAtRelativeSpeed(Percent.per(Second).of(25));
  }

  /**
   * Returns a pattern that scrolls red and white stripes like a candy cane.
   * @return LEDPattern that scrolls red and white stripes
   */
  private LEDPattern candyCanePattern() {
    final int stripeLeds = 6;

    LEDPattern stripes = (reader, writer) -> {
      for (int i = 0; i < reader.getLength(); i++) {
        writer.setLED(i, (i / stripeLeds) % 2 == 0 ? Color.kRed : Color.kWhite);
      }
    };

    return stripes.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), LEDConstants.kLEDSpacing);
  }

  /**
   * Returns a pattern that scrolls through a rainbow of colors like a disco light.
   * @return LEDPattern that scrolls through a rainbow of colors
   */
  private LEDPattern discoPattern() {
    return LEDPattern.gradient(
      GradientType.kContinuous,
      Color.kMagenta, 
      Color.kYellow, 
      Color.kCyan, 
      Color.kHotPink,
      Color.kLime, 
      Color.kOrange, 
      Color.kBlue
    )
    .scrollAtAbsoluteSpeed(MetersPerSecond.of(3.5), LEDConstants.kLEDSpacing);
  }

  /**
   * Returns a pattern that simulates a flame effect with flickering LEDs.
   * Each LED flickers independently, hottest at the base and cooling toward the tip.
   * @return LEDPattern that simulates a flame effect
   */
  private LEDPattern flamePattern() {
    final double heatFloor = 0.08;   // embers: strip never goes fully dark
    final boolean baseAtStart = true; // set false if LED 0 is at the top of your strip

    LEDPattern flame = (reader, writer) -> {
      int len = reader.getLength();
      double t = Timer.getFPGATimestamp();
      double twoPi = 2.0 * Math.PI;

      for (int i = 0; i < len; i++) {
        double pos = len > 1 ? (double) i / (len - 1) : 0.0;
        if (!baseAtStart) {
          pos = 1.0 - pos;
        }

        // Three slow sines at different rates; per-LED phase offsets decorrelate neighbors,
        // and the -pos term makes the flicker drift up the strip.
        double n = Math.sin(twoPi * 0.9 * t + i * 1.7 - pos * 6.0)
                 + Math.sin(twoPi * 1.7 * t + i * 2.9 + pos * 4.0)
                 + Math.sin(twoPi * 2.3 * t + i * 0.6 - pos * 8.0);
        double flicker = 0.5 + n / 6.0; // 0..1

        // Hotter at the base, cooler at the tip, modulated by flicker
        double base = Math.pow(1.0 - pos, 1.3);
        double heat = Math.max(heatFloor, base * (0.55 + 0.45 * flicker));

        // Heat palette: dark red -> red -> orange -> yellow
        double scaled = heat * 3.0;
        double r = MathUtil.clamp(scaled, 0.0, 1.0);
        double g = MathUtil.clamp(scaled - 1.0, 0.0, 1.0) * 0.55;
        double b = MathUtil.clamp(scaled - 2.0, 0.0, 1.0) * 0.15;

        writer.setRGB(i, (int) (255 * r), (int) (255 * g), (int) (255 * b));
      }
    };

    return flame;
  }
  
  /**
   * Returns a pattern that simulates a heartbeat pulse effect with double-pulses in red.
   * @return LEDPattern that simulates a heartbeat pulse effect
   */
  private LEDPattern heartbeatPattern() {
    final double period = 1.2; // ~0.8 beats/sec, 2 pulses each

    LEDPattern beat = (reader, writer) -> {
      double t = (Timer.getFPGATimestamp() % period) / period;
      double lub = Math.exp(-Math.pow((t - 0.10) / 0.05, 2));
      double dub = 0.7 * Math.exp(-Math.pow((t - 0.30) / 0.05, 2));
      double b = Math.max(0.1, Math.max(lub, dub)); // floor so it never goes fully dark
      for (int i = 0; i < reader.getLength(); i++) {
        writer.setRGB(i, (int) (255 * b), 0, 0);
      }
    };

    return beat;
  }

  /**
   * Returns a pattern that simulates streaming comets with white-hot heads and fading blue tails.
   * One or more comets travel along the strip, exit completely, then re-enter.
   * @return LEDPattern that simulates streaming comets
   */
  private LEDPattern meteorPattern() {
    final double speedLedsPerSec = 40.0; // head speed
    final double tailLeds = 14.0;        // tail length
    final int numComets = 2;             // evenly spaced comets in flight
    final boolean forward = true;        // false to stream toward LED 0

    LEDPattern comets = (reader, writer) -> {
      int len = reader.getLength();
      double cycle = len + tailLeds; // head travels far enough for the tail to clear the strip
      double t = Timer.getFPGATimestamp();

      for (int i = 0; i < len; i++) {
        int idx = forward ? i : len - 1 - i;
        double f = 0.0; // brightest comet contribution at this LED

        for (int k = 0; k < numComets; k++) {
          double head = (t * speedLedsPerSec + k * cycle / numComets) % cycle;
          double d = head - idx; // LEDs behind the head (positive = in the tail)

          double b;
          if (d < 0.0) {
            b = Math.max(0.0, 1.0 + d); // ~1 LED soft leading edge
          } else if (d <= tailLeds) {
            b = Math.pow(1.0 - d / tailLeds, 2.0); // quadratic tail falloff
          } else {
            b = 0.0;
          }
          f = Math.max(f, b);
        }

        // Color by intensity: white head -> cyan -> blue -> dark
        double r = MathUtil.clamp((f - 0.7) / 0.3, 0.0, 1.0);
        double g = MathUtil.clamp((f - 0.3) / 0.5, 0.0, 1.0);
        double bl = MathUtil.clamp(f * 1.5, 0.0, 1.0);

        writer.setRGB(i, (int) (255 * r), (int) (255 * g), (int) (255 * bl));
      }
    };

    return comets;
  }

  /**
   * Returns a pattern that simulates a Knight Rider / KITT scanner effect with 
   * a bright red "eye" bouncing back and forth.
   * @return LEDPattern that simulates a Knight Rider / KITT scanner effect
   */
  private LEDPattern knightRiderPattern() {
    final double sweepSeconds = 2.0; // there and back
    final double tailLeds = 6.0;

    LEDPattern kitt = (reader, writer) -> {
      int len = reader.getLength();
      double t = (Timer.getFPGATimestamp() % sweepSeconds) / sweepSeconds;
      double tri = t < 0.5 ? t * 2 : 2 - t * 2; // 0 -> 1 -> 0
      double eye = tri * (len - 1);
      for (int i = 0; i < len; i++) {
        double b = Math.max(0.0, 1.0 - Math.abs(i - eye) / tailLeds);
        writer.setRGB(i, (int) (255 * b), 0, 0);
      }
    };

    return kitt;
  }
  
  /**
   * Returns a pattern that simulates a lava lamp effect with smoothly transitioning colors.
   * @return LEDPattern that simulates a lava lamp effect
   */
  private LEDPattern lavaLampPattern() {
    return LEDPattern.gradient(
      GradientType.kContinuous,
      Color.kPurple, 
      Color.kDarkRed, 
      Color.kDeepPink, 
      Color.kDarkMagenta
    )
    .scrollAtRelativeSpeed(Percent.per(Second).of(8))
    .breathe(Seconds.of(2.5));
  }

  /**
   * Returns a pattern that simulates Matrix-style "digital rain" with several drops streaming 
   * along the strip. Each drop has a bright head and a fading tail, and they move at 
   * different speeds.
   * @return LEDPattern that simulates Matrix-style "digital rain"
   */
  private LEDPattern matrixPattern() {
    final int numDrops = 7;
    final double minSpeed = 12.0, maxSpeed = 40.0; // LEDs per second
    final double minTail = 5.0, maxTail = 14.0;    // tail length in LEDs
    final double maxIdleSec = 2.5;                 // max pause between passes
    final double shimmer = 0.15;                   // per-LED brightness jitter (0..1)
    final double shimmerHz = 3.0;                  // how often the jitter re-rolls
    final double bgGlow = 0.03;                    // faint green floor, never fully dark
    final boolean forward = true;                  // false to fall toward LED 0

    // Fixed per-drop parameters, derived from a hash so the look is irregular but repeatable
    final double[] speed = new double[numDrops];
    final double[] tail = new double[numDrops];
    final double[] idle = new double[numDrops];
    final double[] phase = new double[numDrops]; // 0..1 offset into each drop's cycle
    for (int k = 0; k < numDrops; k++) {
      speed[k] = minSpeed + (maxSpeed - minSpeed) * hash01(k, 1);
      tail[k] = minTail + (maxTail - minTail) * hash01(k, 2);
      idle[k] = maxIdleSec * hash01(k, 3);
      phase[k] = hash01(k, 4);
    }
    final double[] head = new double[numDrops]; // scratch, reused every loop

    LEDPattern rain = (reader, writer) -> {
      int len = reader.getLength();
      double t = Timer.getFPGATimestamp();

      // Head position for each drop; beyond (len + tail) means it's idling off-strip
      for (int k = 0; k < numDrops; k++) {
        double travel = len + tail[k];
        double period = travel / speed[k] + idle[k];
        double local = (t + phase[k] * period) % period;
        head[k] = local * speed[k];
      }

      int bucket = (int) (t * shimmerHz);

      for (int i = 0; i < len; i++) {
        int idx = forward ? i : len - 1 - i;
        double f = 0.0;

        for (int k = 0; k < numDrops; k++) {
          double d = head[k] - idx; // LEDs behind the head (positive = in the tail)
          double b;
          if (d < 0.0) {
            b = Math.max(0.0, 1.0 + d); // ~1 LED soft leading edge
          } else if (d <= tail[k]) {
            b = Math.pow(1.0 - d / tail[k], 1.5);
          } else {
            b = 0.0;
          }
          f = Math.max(f, b);
        }

        // Jitter lit pixels slightly, independently per LED, to suggest changing glyphs
        if (f > 0.0) {
          f *= 1.0 - shimmer * hash01(i, bucket);
        }

        double g = MathUtil.clamp(bgGlow + f, 0.0, 1.0);
        double white = MathUtil.clamp((f - 0.85) / 0.15, 0.0, 1.0) * 0.85; // head turns white-ish
        writer.setRGB(i, (int) (255 * white * g), (int) (255 * g), (int) (255 * white * g));
      }
    };

    return rain;
  }

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the LED subsystem at the start of the autonomous phase.
   */
  public void autonomousInit() {
    cancelCurrentCommand();
    matchPhase = 'A';
    Utils.logInfo("LED subsystem initialized for autonomous");
  }

  /**
   * Initializes the LED subsystem at the start of the teleop phase.
   */
  public void teleopInit() {
    cancelCurrentCommand();
    matchPhase = 'T';
    Utils.logInfo("LED subsystem initialized for teleop");
  }

  /**
   * Initializes the LED subsystem for post match (disabled) state.
   */
  public void postMatch() {
    cancelCurrentCommand();
    matchPhase = 'P';
    Utils.logInfo("LED subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------
  
  /**
   * Creates a command that runs a pattern on the entire LED strip until interrupted. 
   * @param pattern the LED pattern to run
   */
  public Command displayCommand(LEDPattern pattern) {
    LEDPattern dimmed = pattern.atBrightness(LEDConstants.kBrightness);
    return run(() -> displayPattern(dimmed)).ignoringDisable(true);
  }

  /**
   * Returns a command that runs a pattern based on the current match phase.
   * @return Command corresponding to the current match phase
   */
  public Command matchPhaseCommand() {
    // Return a command based on the current match phase
    if (matchPhase == 'A') {
      return matrixCommand();
    } 
    else if (matchPhase == 'T') {
      return allianceCommand();
    } 
    else if (matchPhase == 'P') {
      return teamColorsCommand();
    } 

    // If no match phase is set, default to turning off the LEDs
    return offCommand();
  }

  /**
   * Command to set the LED strip off until another command takes over
   * @return Command that sets LED display off
   */
  public Command offCommand() {
    return displayCommand(offPattern()).withName("LED_Off");
  }
  
  /**
   * Command to indicate success state until interrupted
   * @return Command with success LED
   */
  public Command successCommand() {
    return displayCommand(successPattern()).withName("LED_Success");
  }
  
  /**
   * Command to indicate info state until interrupted
   * @return Command with info LED
   */
  public Command infoCommand() {
    return displayCommand(infoPattern()).withName("LED_Info");
  }
  
  /**
   * Command to indicate warning state until interrupted
   * @return Command with warning LED
   */
  public Command warningCommand() {
    return displayCommand(warningPattern()).withName("LED_Warning");
  }
  
  /**
   * Command to indicate error state until interrupted
   * @return Command with error LED
   */
  public Command errorCommand() {
    return displayCommand(errorPattern()).withName("LED_Error");
  }
  
  /**
   * Command to set idle feedback until interrupted
   * @return Command that sets idle LED display
   */
  public Command idleCommand() {
    return displayCommand(idlePattern()).withName("LED_Idle");
  }
  
  /**
   * Command to display alliance colors (solid red or blue) until interrupted  
   * @return Command that shows the alliance colors
   */
  public Command allianceCommand() {
    return displayCommand(alliancePattern()).withName("LED_AllianceColors");
  }
  
  /**
   * Command to display team colors (scrolling gradient) until interrupted  
   * @return Command that scrolls between green & copper
   */
  public Command teamColorsCommand() {
    return displayCommand(teamColorsPattern()).withName("LED_TeamColors");
  }

  /**
   * Command to display candy cane pattern (scrolling stripes) until interrupted
   * @return Command that shows rotating red and white stripes
   */
  public Command candyCaneCommand() {
    return displayCommand(candyCanePattern()).withName("LED_CandyCane");
  }

  /**
   * Command to display disco pattern (scrolling rainbow) until interrupted
   * @return Command that shows random flashing rainbow colors
   */
  public Command discoCommand() {
    return displayCommand(discoPattern()).withName("LED_Disco");
  }

  /**
   * Command to display a flame effect until interrupted.
   * @return Command that shows an animated fire effect
   */
  public Command flameCommand() {
    return displayCommand(flamePattern()).withName("LED_Flame");
  }

  /**
   * Command to display a heartbeat pulse effect until interrupted.
   * @return Command that shows a heartbeat pulse
   */
  public Command heartbeatCommand() {
    return displayCommand(heartbeatPattern()).withName("LED_Heartbeat");
  }

  /**
   * Command to display streaming comets until interrupted.
   * @return Command that shows streaming comets
   */
  public Command meteorCommand() {
    return displayCommand(meteorPattern()).withName("LED_Meteor");
  }

  /**
   * Command to display a Knight Rider / KITT scanner effect until interrupted.
   * @return Command that shows a scanning red light
   */
  public Command knightRiderCommand() {
    return displayCommand(knightRiderPattern()).withName("LED_KnightRider");
  }

  /**
   * Command to display a lava lamp / slow plasma effect until interrupted.
   * @return Command that shows a hypnotic lava lamp effect
   */
  public Command lavaLampCommand() {
    return displayCommand(lavaLampPattern()).withName("LED_LavaLamp");
  }

  /**
   * Command to display Matrix-style "digital rain" until interrupted.
   * @return Command that shows Matrix-style digital rain
   */
  public Command matrixCommand() {
    return displayCommand(matrixPattern()).withName("LED_Matrix");
  }
}
