// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/**
 * Constants for the turret subsystem.
 */
public final class TurretConstants {
  // ============================================================
  // Turret rotation constants
  // ============================================================

  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------
  public static final double kTurretGearRatio = 1.0; // Motor rotations per turret rotation

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

  // ============================================================
  // Flywheel constants
  // ============================================================

  // ------------------------------------------------------------
  // Physical constants
  // ------------------------------------------------------------
  public static final boolean kFlywheelFollowerOpposesLeader = true; // True if motors are mechanically mirrored

  // ------------------------------------------------------------
  // Control constants
  // ------------------------------------------------------------
  public static final double kFlywheelToleranceRPM    = 50.0;  // RPM window to consider flywheel at target
  public static final double kFlywheelMinSpinningRPM  = 100.0; // RPM threshold to consider flywheel spinning

  // ------------------------------------------------------------
  // Preset velocities - adjust for your game piece and target
  // ------------------------------------------------------------
  public static final double kFlywheelIdleRPM  = 0.0;
  public static final double kFlywheelShortRPM = 2000.0;
  public static final double kFlywheelMidRPM   = 3500.0;
  public static final double kFlywheelLongRPM  = 5000.0;

  // ------------------------------------------------------------
  // PID / Feedforward gains (tune with SysId)
  // Tuning: start with kV only (kP = 0), add kP if error remains
  // ------------------------------------------------------------
  public static final double kFlywheelKP = 0.0;
  public static final double kFlywheelKI = 0.0;
  public static final double kFlywheelKD = 0.0;
  public static final double kFlywheelKS = 0.0; // Static friction - from SysId
  public static final double kFlywheelKV = 0.0; // Velocity feedforward - from SysId
  public static final double kFlywheelKA = 0.0; // Acceleration feedforward - from SysId
}
