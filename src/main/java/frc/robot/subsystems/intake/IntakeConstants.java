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
  // Limits should be based on zero being when the climber is straight up.
  // If the climber is zeroed in a different position, then the chain 
  // tensioner may contact the climber gears and cause damage.
  public static final double kFlywheelKP         = -75.0; // Maximum up was -250
  public static final double kFlywheelKI         = 140.0; // Maximum down was 200
  public static final double kFlywheelKD         = 0.0;   // Derivative term
  public static final double kFlywheelKS         = 0.0;   // Static feedforward
  public static final double kFlywheelKV         = 0.0;   // Velocity feedforward
  public static final double kFlywheelKA         = 0.0;   // Acceleration feedforward  
}
