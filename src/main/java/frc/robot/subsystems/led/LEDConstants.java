// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

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
  public static final int kLEDCount = 60;

  /** Length of the LED strip in meters */
  public static final double kLEDLengthMeters = 1.0;

  /** Distance between LEDs in meters */
  public static final Distance kLEDSpacing = Meters.of(kLEDLengthMeters / kLEDCount);

  /** Control the brightness of the LEDs [0, 100] */
  public static final Dimensionless kBrightness = Percent.of(65);
}
