// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

/**
 * Utility class for creating TalonFX based motor controllers for Krakens, Talons, and Falcons. 
 */
public final class TalonFXFactory {
  // Prevent instantiation
  private TalonFXFactory() {}

  // Define the motor team container
  public record MotorPair(TalonFX leader, TalonFX follower) {}

  /**
   * Applies a TalonFX configuration to a motor with retry logic
   * @param motor The TalonFX motor
   * @param config The TalonFX configuration
   * @return True if the configuration was applied successfully, false otherwise
   */
  public static boolean applyConfig(TalonFX motor, TalonFXConfiguration config) {
    // Apply the configuration to the motor
    StatusCode status = motor.getConfigurator().apply(config);

    // Retry up to 3 times if the configuration fails
    for (int i = 0; i < 3 && status != StatusCode.OK; i++) {
      Utils.logError("Failed to apply TalonFX config, retrying... (" + (i + 1) + "/3)");
      status = motor.getConfigurator().apply(config);
    }

    // Return true if the configuration was applied successfully, false otherwise
    return status == StatusCode.OK;
  }

  /**
   * Creates an independent non-follower TalonFX motor with the given device ID and configuration.
   * @param deviceID The CAN ID that the motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static TalonFX createMotor(int deviceID, TalonFXConfiguration config) {
    return createMotor(deviceID, config, false);
  }

  /**
   * Creates a TalonFX motor with the given device ID and configuration.
   * @param deviceID The CAN ID that the motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @param isFollower Whether the motor is a follower (affects CAN status frame optimization)
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static TalonFX createMotor(int deviceID, TalonFXConfiguration config, boolean isFollower) {
    // Create the motor
    TalonFX motor = new TalonFX(deviceID);

    // Apply the configuration to the motor
    if (!applyConfig(motor, config)) {
      Utils.logError("Failed to apply configuration to TalonFX with device ID: " + deviceID);
    }

    // Optimize the motor's CAN status frames to reduce bus utilization
    optimize(motor, isFollower);

    // Return the motor
    return motor;
  }

  /**
   * Creates a pair of TalonFX motors with the given device IDs and configuration.
   * @param leaderDeviceID The CAN ID that the leader motor is connected to
   * @param followerDeviceID The CAN ID that the follower motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @param followerIsInverted Whether the follower motor is inverted relative to the leader motor
   * @return A pair of TalonFX motors with the given device IDs and configuration
   */
  public static MotorPair createMotorPair(
    int leaderDeviceID, 
    int followerDeviceID,
    TalonFXConfiguration config,
    boolean followerIsInverted
  ) {
    // Create the motors
    TalonFX leaderMotor = createMotor(leaderDeviceID, config, false);
    TalonFX followerMotor = createMotor(followerDeviceID, config, true);

    // Set the follower motor to follow the leader motor
    followerMotor.setControl(new Follower(
      leaderDeviceID, 
      followerIsInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned
    ));
    
    // Return the motors
    return new MotorPair(leaderMotor, followerMotor);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce
   * bus utilization using a set of sensible default values.
   * @param motor The TalonFX motor
   */
  public static void optimize(TalonFX motor) {
    optimize(motor, false);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce 
   * bus utilization using a set of sensible default values.
   * @param motor The TalonFX motor
   * @param isFollower Whether the motor is a follower (affects CAN status frame optimization)
   */
  public static void optimize(TalonFX motor, boolean isFollower) {
    optimize(motor, 50.0, 50.0, isFollower);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using the provided update frequencies for each status frame.
   * @param motor The TalonFX motor
   */
  public static void optimize(
    TalonFX motor, 
    double velocityUpdateHz, 
    double positionUpdateHz, 
    boolean isFollower
  ) {
    // Check if the motor is a follower and apply the appropriate update frequencies
    if (isFollower) {
      motor.getVelocity().setUpdateFrequency(10.0);
      motor.getPosition().setUpdateFrequency(10.0);
      motor.getMotorVoltage().setUpdateFrequency(10.0);
      motor.getSupplyCurrent().setUpdateFrequency(10.0);
      motor.getTorqueCurrent().setUpdateFrequency(10.0);
      motor.getDeviceTemp().setUpdateFrequency(4.0);
    } else {
      motor.getVelocity().setUpdateFrequency(velocityUpdateHz);
      motor.getPosition().setUpdateFrequency(positionUpdateHz);
      motor.getMotorVoltage().setUpdateFrequency(50.0);
      motor.getSupplyCurrent().setUpdateFrequency(50.0);
      motor.getTorqueCurrent().setUpdateFrequency(20.0);
      motor.getDeviceTemp().setUpdateFrequency(4.0);
    }

    // Optimize the motor's CAN status frames to reduce bus utilization
    motor.optimizeBusUtilization();
  }
}
