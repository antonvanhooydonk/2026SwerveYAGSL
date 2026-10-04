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
 * Utility class for creating consistent TalonFX based configurations
 * and motor controllers for Krakens, Talons, and Falcons. 
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
    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 3 && status != StatusCode.OK; i++) {
      Utils.logError("Failed to apply TalonFX config, retrying... (" + (i + 1) + "/3)");
      status = motor.getConfigurator().apply(config);
    }

    return status == StatusCode.OK;
  }

  /**
   * Creates a TalonFX motor with the given device ID and configuration.
   * @param deviceID The CAN ID that the motor is connected to
   * @param config The TalonFX configuration to apply to the motor
   * @return A TalonFX motor with the given device ID and configuration
   */
  public static TalonFX createMotor(int deviceID, TalonFXConfiguration config) {
    // Create the motor
    TalonFX motor = new TalonFX(deviceID);

    // Apply the configuration to the motor
    if (!applyConfig(motor, config)) {
      Utils.logError("Failed to apply configuration to TalonFX with device ID: " + deviceID);
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
    TalonFXConfiguration config,
    boolean followerIsInverted
  ) {
    // Create the motors
    TalonFX leaderMotor = createMotor(leaderDeviceID, config);
    TalonFX followerMotor = createMotor(followerDeviceID, config);

    // Set the follower motor to follow the leader motor
    followerMotor.setControl(new Follower(
      leaderDeviceID, 
      followerIsInverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned
    ));
    
    // Return the motors
    return new MotorPair(leaderMotor, followerMotor);
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using a set of sensible default values.
   * @param motor The TalonFX motor
   */
  public static void optimize(TalonFX motor) {
    optimize(
      motor,
      100.0,
      100.0,
      50.0,
      50.0,
      50.0,
      4.0 
    );
  }

  /**
   * Optimizes the CAN status frames for a TalonFX motor to reduce bus utilization
   * using the provided update frequencies for each status frame.
   * @param motor The TalonFX motor
   */
  public static void optimize(
    TalonFX motor,
    double velocityUpdateFrequency,
    double positionUpdateFrequency,
    double motorVoltageUpdateFrequency,
    double supplyCurrentUpdateFrequency,
    double torqueCurrentUpdateFrequency,
    double deviceTempUpdateFrequency
  ) {
    motor.getVelocity().setUpdateFrequency(velocityUpdateFrequency);
    motor.getPosition().setUpdateFrequency(positionUpdateFrequency);
    motor.getMotorVoltage().setUpdateFrequency(motorVoltageUpdateFrequency);
    motor.getSupplyCurrent().setUpdateFrequency(supplyCurrentUpdateFrequency);
    motor.getTorqueCurrent().setUpdateFrequency(torqueCurrentUpdateFrequency);
    motor.getDeviceTemp().setUpdateFrequency(deviceTempUpdateFrequency);
    motor.optimizeBusUtilization();
  }
}
