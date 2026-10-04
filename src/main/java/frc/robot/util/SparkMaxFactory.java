// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Utility class for creating consistent SparkMax based configurations
 * and motor controllers for NEOs & CIMs. 
 */
public final class SparkMaxFactory {
  // Prevent instantiation
  private SparkMaxFactory() {}

  // Define the motor team container
  public record MotorPair(SparkMax leader, SparkMax follower) {}

  /**
   * Creates and returns a default TalonFX configuration
   * @param smartCurrentLimit The smart current limit for the motor
   * @param idleMode The idle mode for the motor
   * @param isInverted Whether the motor is inverted
   * @param kP The proportional gain for the PID controller
   * @param kI The integral gain for the PID controller
   * @param kD The derivative gain for the PID controller
   * @return A default TalonFX configuration
   */
  public static SparkMaxConfig createConfig(
    int smartCurrentLimit, 
    IdleMode idleMode, 
    boolean isInverted,
    double kP,
    double kI,
    double kD
  ) {
    SparkMaxConfig config = new SparkMaxConfig();
    
    config
      .smartCurrentLimit(smartCurrentLimit) 
      .voltageCompensation(12) 
      .idleMode(idleMode)
      .inverted(isInverted);

    config.closedLoop
      .p(kP)
      .i(kI)
      .d(kD);

    return config;
  }

  /**
   * Applies motion control parameters to a SparkMax configuration.
   * The config is modified in place (pass by reference) and returned for convenience.
   * @param config The configuration to apply the motion control parameters to
   * @param forwardLimitRotations The maximum forward limit in motor rotations
   * @param reverseLimitRotations The maximum reverse limit in motor rotations
   * @return The updated SparkMax configuration with the motion control parameters applied
   */
  public static SparkMaxConfig setSoftwareLimits(
    SparkMaxConfig config,
    double forwardLimitRotations,
    double reverseLimitRotations
  ) {
    config.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(forwardLimitRotations)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(reverseLimitRotations);

    return config;
  }

  /**
   * Applies motion control parameters to a SparkMax configuration.
   * The config is modified in place (pass by reference) and returned for convenience.
   * @param config The configuration to apply the motion control parameters to
   * @param cruiseVelocityRPM The maximum velocity for the motion control
   * @param accelerationRPM2 The maximum acceleration for the motion control
   * @param toleranceRots The maximum tolerance for the motion control
   * @return The updated SparkMax configuration with the motion control parameters applied
   */
  public static SparkMaxConfig setMotionControl(
    SparkMaxConfig config,
    double cruiseVelocityRPM,
    double accelerationRPM2,
    double toleranceRotations
  ) {
    config.closedLoop.maxMotion
      .cruiseVelocity(cruiseVelocityRPM)
      .maxAcceleration(accelerationRPM2)
      .allowedProfileError(toleranceRotations);

    return config;
  }

  /**
   * Applies a TalonFX configuration to a motor with retry logic.
   * This method will replace any existing configuration applied to the motor.
   * @param motor The TalonFX motor
   * @param config The TalonFX configuration
   * @return True if the configuration was applied successfully, false otherwise
   */
  public static boolean applyConfig(SparkMax motor, SparkMaxConfig config) {
    REVLibError err = motor.configure(
      config, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    for (int i = 0; i < 3 && err != REVLibError.kOk; i++) {
      Utils.logError("Failed to apply SparkMax config, retrying... (" + (i + 1) + "/3)");
      err = motor.configure(
        config, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters
      );
    }

    return err == REVLibError.kOk;
  }

  /**
   * Optimizes the CAN status frames for a SparkMax motor to reduce bus utilization
   * using a set of sensible default values.
   * @param motor The SparkMax motor
   */
  public static void optimize(SparkMax motor) {
    optimize(
      motor,
      20,
      20,
      500,
      500,
      500,
      200,
      500
    );
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using the provided update frequencies for each status frame.
   * @param motor The TalonFX motor
   */
  public static void optimize(
    SparkMax motor,
    int primaryEncoderPositionPeriodMs,
    int primaryEncoderVelocityPeriodMs,
    int externalOrAltEncoderPositionMs,
    int externalOrAltEncoderVelocityMs,
    int appliedOutputPeriodMs,
    int faultsPeriodMs,
    int analogVoltagePeriodMs
  ) {
    SparkMaxConfig signals = new SparkMaxConfig();
    
    signals.signals
      .primaryEncoderPositionPeriodMs(primaryEncoderPositionPeriodMs)
      .primaryEncoderVelocityPeriodMs(primaryEncoderVelocityPeriodMs)
      .externalOrAltEncoderPosition(externalOrAltEncoderPositionMs)
      .externalOrAltEncoderVelocity(externalOrAltEncoderVelocityMs)
      .appliedOutputPeriodMs(appliedOutputPeriodMs)
      .faultsPeriodMs(faultsPeriodMs)
      .analogVoltagePeriodMs(analogVoltagePeriodMs); 

    motor.configure(
      signals, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  /**
   * Creates a SparkMax brushless motor with the given device ID and configuration.
   * @param deviceID The CAN ID of the motor
   * @param config The SparkMax configuration to apply to the motor
   * @return The SparkMax motor with the given device ID and configuration
   */
  public static SparkMax createMotor(int deviceID, SparkMaxConfig config) {
    return createMotor(deviceID, MotorType.kBrushless, config);
  }

  /**
   * Creates a TalonFX motor with the given device ID and configuration.
   * @param deviceID The CAN ID that the motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static SparkMax createMotor(int deviceID, MotorType motorType, SparkMaxConfig config) {
    // Create the motor
    SparkMax motor = new SparkMax(deviceID, motorType);

    // Apply the configuration to the motor
    if (!applyConfig(motor, config)) {
      Utils.logError("Failed to apply configuration to SparkMax with device ID: " + deviceID);
    }

    // Optimize the motor's CAN status frames to reduce bus utilization
    optimize(motor);

    // Return the motor
    return motor;
  }

  /**
   * Creates a TalonFX motor with the given device ID and configuration.
   * @param leaderDeviceID The CAN ID that the leader motor is connected to
   * @param followerDeviceID The CAN ID that the follower motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @param followerIsInverted Whether the follower motor is inverted relative to the leader motor
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static MotorPair createMotorPair(
    int leaderDeviceID, 
    int followerDeviceID,
    SparkMaxConfig config,
    boolean followerIsInverted
  ) {
    // Create the motors
    SparkMax leaderMotor = createMotor(leaderDeviceID, config);

    // Create the follower motor configuration based on the leader motor configuration
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
      .apply(config)
      .follow(leaderMotor)
      .inverted(followerIsInverted);
    
    // Create the follower motor
    SparkMax followerMotor = createMotor(followerDeviceID, followerConfig);
    
    // Return the motors
    return new MotorPair(leaderMotor, followerMotor);
  }
}
