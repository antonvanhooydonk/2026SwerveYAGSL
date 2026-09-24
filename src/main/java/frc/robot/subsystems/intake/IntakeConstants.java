// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/**
 * Constants for the Intake subsystem
 * All values should be tuned based on your specific robot
 */
public final class IntakeConstants {
  // Motor power percentages
  public static final double kRollerForwardRPM =  2000;  // Power for climbing up
  public static final double kRollerReverseRPM = -2000;  // Power for climbing down (negative)
  
  // It is crucial to set these limits correctly to prevent mechanical damage. 
  public static final double kRollerKP = 1.0; // Maximum up was -250
  public static final double kRollerKI = 0.0; // Maximum down was 200
  public static final double kRollerKD = 0.0;   // Derivative term
  public static final double kRollerKS = 0.0;   // Static feedforward
  public static final double kRollerKV = 0.0;   // Velocity feedforward
  public static final double kRollerKA = 0.0;   // Acceleration feedforward 
 
}
