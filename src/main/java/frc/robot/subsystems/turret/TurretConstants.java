// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/**
 * Constants for the turret subsystem.
 */
public final class TurretConstants {
  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------
  public static final double kTurretGearRatio = 1.0; // motor shaft rotations per one full 360 deg turret rotation
  public static final double kMinAngleDegrees = 0.0; // minimum safe raw (unwrapped) turret position, in degrees
  public static final double kMaxAngleDegrees = 330.0; // maximum safe raw (unwrapped) turret position, in degrees
  public static final double kHomeTimeoutSeconds = 5.0; // timeout for homing the turret, in seconds

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kTurretAngleToleranceDegrees = 1.0;

  // ------------------------------------------------------------
  // MotionMagic constraints
  // ------------------------------------------------------------
  public static final double kTurretCruiseVelocityDPS = 180.0; // degrees per second
  public static final double kTurretAccelerationDPS2  = 360.0; // degrees per second squared
  public static final double kTurretJerkDPS3          = 3600.0; // degrees per second cubed

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // ------------------------------------------------------------
  public static final double kTurretKP = 0.1;
  public static final double kTurretKI = 0.0;
  public static final double kTurretKD = 0.0;
  public static final double kTurretKS = 0.0;
  public static final double kTurretKV = 0.0;
  public static final double kTurretKA = 0.0;
}
