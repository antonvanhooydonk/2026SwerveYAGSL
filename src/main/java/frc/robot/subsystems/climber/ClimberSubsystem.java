// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class ClimberSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware (we're using a brushed CIM for the climber)
    climberMotor = new SparkMax(CANConstants.kClimberMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();
    
    // Initialize encoder
    climberEncoder = climberMotor.getEncoder();

    // Reset the encoder (assumes climber starts at the home position)
    resetEncoder();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Climber", this);
    
    // Output initialization progress
    Utils.logInfo("Climber subsystem initialized");
  }
  
  /**
   * Configure the climber motor with all settings
   */
  private void configureMotor() {
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    // configure the climber motor
    climbConfig
      .smartCurrentLimit(30) // amps
      .voltageCompensation(12) // Consistent behavior across battery voltage
      .idleMode(IdleMode.kBrake); // CRITICAL: Brake mode prevents falling
      
    climbConfig.encoder
      .countsPerRevolution(ClimberConstants.kEncoderTicksPerRevolution) 
      .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
      .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

    // Optimize CAN status frames for reduced lag
    climbConfig.signals
      .primaryEncoderPositionPeriodMs(20)    // Fast position data
      .primaryEncoderVelocityPeriodMs(20)    // Fast velocity data
      .externalOrAltEncoderPosition(500)     // Not used
      .externalOrAltEncoderVelocity(500)     // Not used
      .appliedOutputPeriodMs(500)            // Not needed for open-loop control
      .faultsPeriodMs(200)                   // Keep at 200ms for fault detection
      .analogVoltagePeriodMs(500);           // Not used

    // apply configuration
    climberMotor.configure(
      climbConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {}
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set climber motor power with safety limits
   * @param power Power to apply (-1.0 to 1.0)
   */
  private void setPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1, 1);

    // Safety: Stop at limits to prevent damage
    if (isAtUpperLimit() && clampedPower > 0) {
      stop();
      return;
    }    
    if (isAtLowerLimit() && clampedPower < 0) {
      stop();
      return;
    }
    
    // If within limits, set the motor power
    climberMotor.set(clampedPower);
  }
  
  /**
   * Reset the encoder position to zero.
   * This should only be called when the climber is physically in the "home" position.
   */
  private void resetEncoder() {
    climberEncoder.setPosition(0);
  }

  /**
   * Stop the climber motor immediately
   */
  private void stop() {
    climberMotor.stopMotor();
  }
  
  // ==================== State Methods ====================
  
  /**
   * Get the current position of the climber
   * @return Position in degrees
   */
  private double getPosition() {
    return climberEncoder.getPosition();
  }
  
  /**
   * Get the current speed of the climber
   * @return Speed in degrees per second
   */
  private double getSpeed() {
    return climberEncoder.getVelocity();
  }

  /**
   * Get the current draw of the climber motor
   * @return Current in amps
   */
  private double getCurrent() {
    return climberMotor.getOutputCurrent();
  }
  
  /**
   * Get the temperature of the climber motor
   * @return Temperature in Celsius
   */
  private double getTemperature() {
    return climberMotor.getMotorTemperature();
  }
  
  /**
   * Check if climber is at or above upper position limit
   * @return true if at or past upper limit
   */
  private boolean isAtUpperLimit() {
    return getPosition() <= ClimberConstants.kUpperLimitDegrees;
  }
  
  /**
   * Check if climber is at or below lower position limit
   * @return true if at or past lower limit
   */
  private boolean isAtLowerLimit() {
    return getPosition() >= ClimberConstants.kLowerLimitDegrees;
  }
  
  /**
   * Check if climber is at home position
   * @return true if within tolerance of home position
   */
  private boolean isAtHomePosition() {
    return MathUtil.isNear(
      ClimberConstants.kHomeDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at level 1 climb position
   * @return true if within tolerance of level 1 climb position
   */
  private boolean isAtLevelOneClimbPosition() {
    return MathUtil.isNear(
      ClimberConstants.kLevelOneClimbDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at level 2 climb position
   * @return true if within tolerance of level 2 climb position
   */
  private boolean isAtLevelTwoClimbPosition() {
    return MathUtil.isNear(
      ClimberConstants.kLevelTwoClimbDegrees,
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is stalled (high current, low velocity)
   * Useful for detecting when climber hits a hard stop
   * @return true if motor appears stalled
   */
  private boolean isStalled() {
    return Math.abs(getCurrent()) > ClimberConstants.kStallCurrentThreshold &&
           Math.abs(climberEncoder.getVelocity()) < ClimberConstants.kStallVelocityThreshold;
  }

  /**
   * Initialize the climber for autonomous mode. This resets the encoder zero position.
   * This should be called at the start of autonomous to ensure the drive is in a known state.
   * The climber should be physically positioned at the home position before this is called, 
   * as it does not have limit switches and relies on the encoder zero for accurate positioning.
   */
  public void autonomousInit() {
    resetEncoder();
  }

  // ==================== State Triggers ====================

  /**
   * Fires when climber reaches upper limit
   */
  public final Trigger isAtUpperLimitTrigger = new Trigger(this::isAtUpperLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  /**
   * Fires when climber reaches lower limit
   */
  public final Trigger isAtLowerLimitTrigger = new Trigger(this::isAtLowerLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when climber is at level 1 climb position
   */
  public final Trigger isAtLevelOneClimbPositionTrigger = new Trigger(this::isAtLevelOneClimbPosition)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when climber is at level 2 climb position
   */
  public final Trigger isAtLevelTwoClimbPositionTrigger = new Trigger(this::isAtLevelTwoClimbPosition)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when climber is stalled
   */
  public final Trigger isStalledTrigger = new Trigger(this::isStalled)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  // ==================== Command Factories ====================  
  
  /**
   * Command to stop the climber
   * @return Command that stops the climber motor
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Climber_Stop");
  }

  /**
   * Command to extend the climber upward
   * @return Command that runs climber up at configured speed
   */
  public Command upCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .withName("Climber_Up");
  }
  
  /**
   * Command to retract the climber downward
   * @return Command that runs climber down at configured speed
   */
  public Command downCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .withName("Climber_Down");
  }
  
  /**
   * Command the climber up until upper limit
   * @return Command that rotates to upper limit then stops
   */
  public Command upToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kUpPercent))
      .until(this::isAtUpperLimit)
      .finallyDo(this::stop)
      .withName("Climber_UpToLimit");
  }
  
  /**
   * Command the climber down until lower limit
   * @return Command that rotates to lower limit then stops
   */
  public Command downToLimitCommand() {
    return run(() -> setPower(ClimberConstants.kDownPercent))
      .until(this::isAtLowerLimit)
      .finallyDo(this::stop)
      .withName("Climber_DownToLimit");
  }
  
  /**
   * Command the climber to move to the upright home position
   * @return Command that moves to the home position then stops
   */
  public Command homeCommand() {
    return run(() -> {
      if (getPosition() < ClimberConstants.kHomeDegrees) {
        setPower(ClimberConstants.kDownPercent);
      } else {
        setPower(ClimberConstants.kUpPercent);
      }
    })
    .until(this::isAtHomePosition)
    .finallyDo(this::stop)
    .withName("Climber_Home");
  }
  
  /**
   * Command the climber to move to the level one climb position
   * @return Command that moves to the level one climb position then stops
   */
  public Command levelOneClimbCommand() {
    return run(() -> {
      if (getPosition() < ClimberConstants.kLevelOneClimbDegrees) {
        setPower(ClimberConstants.kDownPercent);
      } else {
        setPower(ClimberConstants.kUpPercent);
      }
    })
    .until(this::isAtLevelOneClimbPosition)
    .finallyDo(this::stop)
    .withName("Climber_LevelOneClimb");
  }
  
  /**
   * Command the climber to move to the level two climb position
   * @return Command that moves to the level two climb position then stops
   */
  public Command levelTwoClimbCommand() {
    return run(() -> {
      if (getPosition() < ClimberConstants.kLevelTwoClimbDegrees) {
        setPower(ClimberConstants.kDownPercent);
      } else {
        setPower(ClimberConstants.kUpPercent);
      }
    })
    .until(this::isAtLevelTwoClimbPosition)
    .finallyDo(this::stop)
    .withName("Climber_LevelTwoClimb");
  }
  
  /**
   * Command to reset the encoder to zero at the current position
   * @return Command that resets the encoder
   */
  public Command setHomePositionCommand() {
    return runOnce(this::resetEncoder)
      .ignoringDisable(true)
      .withName("Climber_SetHomePosition");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ClimberSubsystem");
    builder.addDoubleProperty("Position (deg)", () -> Utils.showDouble(getPosition()), null);
    builder.addDoubleProperty("Velocity (deg per sec)", () -> Utils.showDouble(getSpeed()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
    builder.addBooleanProperty("At Upper Limit", this::isAtUpperLimit, null);
    builder.addBooleanProperty("At Lower Limit", this::isAtLowerLimit, null);
    builder.addBooleanProperty("At Home Position", this::isAtHomePosition, null);
    builder.addBooleanProperty("Stalled", this::isStalled, null);
  }
}
