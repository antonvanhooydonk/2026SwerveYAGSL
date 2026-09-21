// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

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

public class ArmSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax armMotor;
  private final RelativeEncoder armEncoder;

  // Used for cutting power in the event of a stall to prevent damage
  private final Debouncer stallDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
  private double lastAppliedPowerSign = 0;
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Initialize hardware (we're using a brushed CIM for the arm)
    armMotor = new SparkMax(CANConstants.kArmMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();
    
    // Initialize encoder
    armEncoder = armMotor.getEncoder();

    // Reset the encoder (assumes arm starts at the home position)
    resetEncoder();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Initialize dashboard
    SmartDashboard.putData("Arm", this);
    
    // Output initialization progress
    Utils.logInfo("Arm subsystem initialized");
  }
  
  /**
   * Configure the arm motor with all settings
   */
  private void configureMotor() {
    SparkMaxConfig armConfig = new SparkMaxConfig();

    // configure the arm motor
    armConfig
      .smartCurrentLimit(30) // amps
      .voltageCompensation(12) // Consistent behavior across battery voltage
      .idleMode(IdleMode.kBrake); // CRITICAL: Brake mode prevents falling
      
    armConfig.encoder
      .countsPerRevolution(ArmConstants.kEncoderTicksPerRevolution) 
      .positionConversionFactor(ArmConstants.kPositionConversionFactor)
      .velocityConversionFactor(ArmConstants.kVelocityConversionFactor);

    // Optimize CAN status frames for reduced lag
    armConfig.signals
      .primaryEncoderPositionPeriodMs(20)    // Fast position data
      .primaryEncoderVelocityPeriodMs(20)    // Fast velocity data
      .externalOrAltEncoderPosition(500)     // Not used
      .externalOrAltEncoderVelocity(500)     // Not used
      .appliedOutputPeriodMs(500)            // Not needed for open-loop control
      .faultsPeriodMs(200)                   // Keep at 200ms for fault detection
      .analogVoltagePeriodMs(500);           // Not used

    // apply configuration
    armMotor.configure(
      armConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  @Override
  public void periodic() {}
    
  // ==================== Internal State Modifiers ====================
  
  /**
   * Set arm motor power with safety limits
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

    // Safety: if stalled, only block continuing to push in the SAME direction
    // that caused the stall -- still allow reversing to back off a jam
    boolean stalled = stallDebouncer.calculate(isStalled());
    if (stalled && Math.signum(clampedPower) == lastAppliedPowerSign) {
      stop();
      return;
    }
    
    // If within limits, set the motor power
    armMotor.set(clampedPower);
  }
  
  /**
   * Reset the encoder position to zero.
   * This should only be called when the arm is physically in the "home" position.
   */
  private void resetEncoder() {
    armEncoder.setPosition(0);
  }

  /**
   * Stop the arm motor immediately
   */
  private void stop() {
    armMotor.stopMotor();
  }
  
  // ==================== State Methods ====================
  
  /**
   * Get the current position of the arm
   * @return Position in degrees
   */
  private double getPosition() {
    return armEncoder.getPosition();
  }
  
  /**
   * Get the current speed of the arm
   * @return Speed in degrees per second
   */
  private double getSpeed() {
    return armEncoder.getVelocity();
  }

  /**
   * Get the current draw of the arm motor
   * @return Current in amps
   */
  private double getCurrent() {
    return armMotor.getOutputCurrent();
  }
  
  /**
   * Get the temperature of the arm motor
   * @return Temperature in Celsius
   */
  private double getTemperature() {
    return armMotor.getMotorTemperature();
  }
  
  /**
   * Check if arm is at or above upper position limit
   * @return true if at or past upper limit
   */
  private boolean isAtUpperLimit() {
    return getPosition() >= ArmConstants.kUpperLimitDegrees;
  }
  
  /**
   * Check if arm is at or below lower position limit
   * @return true if at or past lower limit
   */
  private boolean isAtLowerLimit() {
    return getPosition() <= ArmConstants.kLowerLimitDegrees;
  }
  
  /**
   * Check if arm is at home position
   * @return true if within tolerance of home position
   */
  private boolean isAtHomePosition() {
    return MathUtil.isNear(
      ArmConstants.kHomeDegrees,
      getPosition(),
      ArmConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if arm is at level 1 climb position
   * @return true if within tolerance of level 1 climb position
   */
  private boolean isAtPositionOne() {
    return MathUtil.isNear(
      ArmConstants.kLevelOneClimbDegrees,
      getPosition(),
      ArmConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if arm is at level 2 climb position
   * @return true if within tolerance of level 2 climb position
   */
  private boolean isAtPositionTwo() {
    return MathUtil.isNear(
      ArmConstants.kLevelTwoClimbDegrees,
      getPosition(),
      ArmConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if arm is stalled (high current, low velocity)
   * Useful for detecting when arm hits a hard stop
   * @return true if motor appears stalled
   */
  private boolean isStalled() {
    return Math.abs(getCurrent()) > ArmConstants.kStallCurrentThreshold &&
           Math.abs(armEncoder.getVelocity()) < ArmConstants.kStallVelocityThreshold;
  }

  /**
   * Initialize the arm for autonomous mode. This resets the encoder zero position.
   * This should be called at the start of autonomous to ensure the drive is in a known state.
   * The arm should be physically positioned at the home position before this is called, 
   * as it does not have limit switches and relies on the encoder zero for accurate positioning.
   */
  public void autonomousInit() {
    resetEncoder();
  }

  // ==================== State Triggers ====================

  /**
   * Fires when arm reaches upper limit
   */
  public final Trigger isAtUpperLimitTrigger = new Trigger(this::isAtUpperLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  /**
   * Fires when arm reaches lower limit
   */
  public final Trigger isAtLowerLimitTrigger = new Trigger(this::isAtLowerLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when arm is at level 1 climb position
   */
  public final Trigger isAtPositionOneTrigger = new Trigger(this::isAtPositionOne)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when arm is at level 2 climb position
   */
  public final Trigger isAtPositionTwoTrigger = new Trigger(this::isAtPositionTwo)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  /**
   * Fires when arm is stalled
   */
  public final Trigger isStalledTrigger = new Trigger(this::isStalled)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  // ==================== Command Factories ====================  
  
  /**
   * Command to stop the arm
   * @return Command that stops the arm motor
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Arm_Stop");
  }

  /**
   * Command to extend the arm upward
   * @return Command that runs arm up at configured speed
   */
  public Command clockwiseCommand() {
    return run(() -> setPower(ArmConstants.kUpPercent))
      .withName("Arm_Clockwise");
  }
  
  /**
   * Command to retract the arm downward
   * @return Command that runs arm down at configured speed
   */
  public Command counterClockwiseCommand() {
    return run(() -> setPower(ArmConstants.kDownPercent))
      .withName("Arm_CounterClockwise");
  }
  
  /**
   * Command the arm up until upper limit
   * @return Command that rotates to upper limit then stops
   */
  public Command positionOneCommand() {
    return run(() -> setPower(ArmConstants.kUpPercent))
      .until(this::isAtPositionOne)
      .finallyDo(this::stop)
      .withName("Arm_PositionOne");
  }
  
  /**
   * Command the arm down until lower limit
   * @return Command that rotates to lower limit then stops
   */
  public Command positionTwoCommand() {
    return run(() -> setPower(ArmConstants.kDownPercent))
      .until(this::isAtPositionTwo)
      .finallyDo(this::stop)
      .withName("Arm_PositionTwo");
  }
  
  /**
   * Command the arm to move to the upright home position
   * @return Command that moves to the home position then stops
   */
  public Command homeCommand() {
    return run(() -> {
      if (getPosition() < ArmConstants.kHomeDegrees) {
        setPower(ArmConstants.kDownPercent);
      } else {
        setPower(ArmConstants.kUpPercent);
      }
    })
    .until(this::isAtHomePosition)
    .finallyDo(this::stop)
    .withName("Arm_Home");
  }

  /**
   * Command to reset the encoder to zero at the current position
   * @return Command that resets the encoder
   */
  public Command setHomePositionCommand() {
    return runOnce(this::resetEncoder)
      .ignoringDisable(true)
      .withName("Arm_SetHomePosition");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ArmSubsystem");
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
