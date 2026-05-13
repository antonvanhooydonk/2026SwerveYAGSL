// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class LEDConstants {
  /** Number of LEDs in the strip */
  public static final int kLEDLength = 60;

  /** Control the brightness of the LEDs 0.0 to 1.0 */
  public static final double kBrightness = 0.85;
      
  /** Color when robot is idle/ready (soft blue) */
  public static final Color kIdle = new Color(0.0, 0.3, 1.0);
      
  /** Color for informational messages (cyan) */
  public static final Color kInfo = new Color(0.0, 0.3, 1.0);

  /** Color for success (green) */
  public static final Color kSuccess = Color.kGreen;

  /** Color for warnings (orange) */
  public static final Color kWarning = new Color(1.0, 0.5, 0.0);

  /** Color for errors (red) */
  public static final Color kError = Color.kRed;
      
  /** Team color - Green */
  public static final Color kTeamGreen = new Color(0.0, 1.0, 0.0);

  /** Team color - Copper */
  public static final Color kTeamCopper = new Color(255.0, 38.0, 0.0);
}
