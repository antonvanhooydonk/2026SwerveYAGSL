// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
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
   * Creates a SparkMax brushless motor with the given device ID and configuration.
   * @param deviceID The CAN ID of the motor
   * @param config The SparkMax configuration to apply to the motor
   * @return The SparkMax motor with the given device ID and configuration
   */
  public static SparkMax createMotor(int deviceID, SparkMaxConfig config) {
    return createMotor(deviceID, MotorType.kBrushless, config, false);
  }

  /**
   * Creates a SparkMax brushed motor with the given device ID and configuration.
   * @param deviceID The CAN ID of the motor
   * @param config The SparkMax configuration to apply to the motor
   * @return The SparkMax motor with the given device ID and configuration
   */
  public static SparkMax createBrushedMotor(int deviceID, SparkMaxConfig config) {
    return createMotor(deviceID, MotorType.kBrushed, config, false);
  }

  /**
   * Creates a TalonFX motor with the given device ID and configuration.
   * @param deviceID The CAN ID that the motor is connected to
   * @param motorType The type of motor (brushless or brushed)
   * @param config The TalonFX configuration to apply to the motor
   * @param isFollower Whether the motor is a follower (affects CAN status frame optimization)
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static SparkMax createMotor(
    int deviceID, 
    MotorType motorType, 
    SparkMaxConfig config, 
    boolean isFollower
  ) {
    // Create the motor
    SparkMax motor = new SparkMax(deviceID, motorType);

    // Apply the configuration to the motor
    if (!applyConfig(motor, config)) {
      Utils.logError("Failed to apply configuration to SparkMax with device ID: " + deviceID);
    }

    // Optimize the motor's CAN status frames to reduce bus utilization
    // Velocity and position updates are set to 50 Hz for leader motors, 
    // and 10 Hz for follower motors
    optimize(motor, isFollower);

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
    return createMotorPair(leaderDeviceID, followerDeviceID, MotorType.kBrushless, config, followerIsInverted);
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
    MotorType motorType, 
    SparkMaxConfig config,
    boolean followerIsInverted
  ) {
    // Create the motors
    SparkMax leaderMotor = createMotor(leaderDeviceID, motorType, config, false);

    // Create the follower motor configuration based on the leader motor configuration
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
      .apply(config)
      .follow(leaderMotor)
      .inverted(followerIsInverted);
    
    // Create the follower motor
    SparkMax followerMotor = createMotor(followerDeviceID, motorType, followerConfig, true);
    
    // Return the motors
    return new MotorPair(leaderMotor, followerMotor);
  }

  /**
   * Optimizes the CAN status frames for a SparkMax motor to reduce bus utilization
   * using a set of sensible default values.
   * @param motor The SparkMax motor
   */
  public static void optimize(SparkMax motor) {
    optimize(motor, false);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using a set of sensible default values.
   * @param motor The TalonFX motor
   * @param isFollower Whether the motor is a follower (affects CAN status frame optimization)
   */
  public static void optimize(SparkMax motor, boolean isFollower) {
    optimize(motor, 20, 20, isFollower);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using the provided update frequencies for each status frame.
   * @param motor The TalonFX motor
   * @param primaryEncoderPositionPeriodMs The update period for the primary encoder position in milliseconds
   * @param primaryEncoderVelocityPeriodMs The update period for the primary encoder velocity in milliseconds
   * @param isFollower Whether the motor is a follower (affects CAN status frame optimization
   */
  public static void optimize(
    SparkMax motor,
    int primaryEncoderPositionPeriodMs,
    int primaryEncoderVelocityPeriodMs,
    boolean isFollower
  ) {
    SparkMaxConfig signals = new SparkMaxConfig();
    
    if (isFollower) {
      signals.signals
        .primaryEncoderPositionPeriodMs(200)
        .primaryEncoderVelocityPeriodMs(200)
        .externalOrAltEncoderPosition(500)
        .externalOrAltEncoderVelocity(500)
        .appliedOutputPeriodMs(500)
        .faultsPeriodMs(200)
        .analogVoltagePeriodMs(500); 
    } else {
      signals.signals
        .primaryEncoderPositionPeriodMs(primaryEncoderPositionPeriodMs)
        .primaryEncoderVelocityPeriodMs(primaryEncoderVelocityPeriodMs)
        .externalOrAltEncoderPosition(500)
        .externalOrAltEncoderVelocity(500)
        .appliedOutputPeriodMs(500)
        .faultsPeriodMs(200)
        .analogVoltagePeriodMs(500); 
    }

    // Apply the configuration to the motor
    motor.configure(
      signals, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }
}
