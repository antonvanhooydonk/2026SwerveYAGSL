// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class ClimberSubsystem extends SubsystemBase {
  // Hardware
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberController;

  // Cached target (for telemetry)
  private double targetAngleDegrees = 0.0;

  // SysId routine
  private final SysIdRoutine sysIdRoutine;
 
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // Initialize hardware (we're using a brushed CIM for the climber)
    climberMotor = new SparkMax(CANConstants.kClimberMotorID, MotorType.kBrushed);
    
    // Configure motor
    configureMotor();

    // Initialize closed-loop controller
    climberController = climberMotor.getClosedLoopController();
    
    // Initialize encoder
    climberEncoder = climberMotor.getEncoder();

    // Reset the encoder (assumes climber starts at the home position)
    resetEncoder();

    // Initialize SysId routine (leader motor only)
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("climber-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> climberMotor.setVoltage(volts),
        null,
        this
      )
    );
    
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

    climbConfig.closedLoop
      .p(ClimberConstants.kClimberKP)
      .i(ClimberConstants.kClimberKI)
      .d(ClimberConstants.kClimberKD)
      .outputRange(-1, 1)
      .maxMotion
        .cruiseVelocity(ClimberConstants.kMaxVelocityDegPerSec)
        .maxAcceleration(ClimberConstants.kMaxAccelDegPerSec2)
        .allowedProfileError(ClimberConstants.kPositionToleranceDegrees);

    climbConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(ClimberConstants.kUpperLimitDegrees)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(ClimberConstants.kLowerLimitDegrees);
      
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
    
  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------
  
  /**
   * Set the target position for the climber with safety limits
   * @param degrees Target position in degrees
   */
  private void setPosition(double degrees) {
    // Clamp target to valid range
    targetAngleDegrees = MathUtil.clamp(
      degrees, 
      ClimberConstants.kLowerLimitDegrees, 
      ClimberConstants.kUpperLimitDegrees
    );

    // Set the target position using max motion control
    climberController.setSetpoint(targetAngleDegrees, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Set the voltage for the climber motor (open-loop control)
   * @param volts Voltage to apply to the motor
   */
  private void setVoltage(double volts) {
    // Clamp voltage to safe range
    volts = MathUtil.clamp(volts, -12, 12);

    // Check if the climber is at the upper limit and trying to move up
    if (isAtUpperLimit() && volts > 0) {
      stop();
      return;
    }

    // Check if the climber is at the lower limit and trying to move down
    if (isAtLowerLimit() && volts < 0) {
      stop();
      return;
    }

    // Apply voltage to the motor
    climberMotor.setVoltage(volts);
  }
    
  /**
   * Get the current position of the climber
   * @return Position in degrees
   */
  private double getPosition() {
    return climberEncoder.getPosition();
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

  /**
   * Normalizes an angle to (-180, 180]
   * @param angleDegrees Angle in degrees
   * @return Normalized angle in degrees
   */
  private double normalizeAngleDegrees(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180.0, 180.0);
  }

  /**
   * Check if climber is at a specific target position within tolerance
   * @param targetDegrees The target position in degrees to check against
   * @return true if within tolerance of the target position  
   */
  private boolean isAtPosition(double targetDegrees) {
    return MathUtil.isNear(
      normalizeAngleDegrees(targetDegrees),
      getPosition(),
      ClimberConstants.kPositionToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at or above upper position limit
   * @return true if at or past upper limit
   */
  private boolean isAtUpperLimit() {
    return isAtPosition(ClimberConstants.kUpperLimitDegrees);
  }
  
  /**
   * Check if climber is at or below lower position limit
   * @return true if at or past lower limit
   */
  private boolean isAtLowerLimit() {
    return isAtPosition(ClimberConstants.kLowerLimitDegrees);
  }
  
  /**
   * Check if climber is at home position
   * @return true if within tolerance of home position
   */
  private boolean isAtHomePosition() {
    return isAtPosition(ClimberConstants.kHomeDegrees);
  }
  
  /**
   * Check if climber is at level 1 climb position
   * @return true if within tolerance of level 1 climb position
   */
  private boolean isAtLevelOneClimbPosition() {
    return isAtPosition(ClimberConstants.kLevelOneClimbDegrees);
  }
  
  /**
   * Check if climber is at level 2 climb position
   * @return true if within tolerance of level 2 climb position
   */
  private boolean isAtLevelTwoClimbPosition() {
    return isAtPosition(ClimberConstants.kLevelTwoClimbDegrees);
  }
  
  /**
   * Check if climber is stalled (high current, low velocity)
   * Useful for detecting when climber hits a hard stop
   * @return true if motor appears stalled
   */
  private boolean isStalled() {
    return Math.abs(climberMotor.getOutputCurrent()) > ClimberConstants.kStallCurrentThreshold &&
           Math.abs(climberEncoder.getVelocity()) < ClimberConstants.kStallVelocityThreshold;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isAtUpperLimitTrigger = new Trigger(this::isAtUpperLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLowerLimitTrigger = new Trigger(this::isAtLowerLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelOneClimbPositionTrigger = new Trigger(this::isAtLevelOneClimbPosition)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelTwoClimbPositionTrigger = new Trigger(this::isAtLevelTwoClimbPosition)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isStalledTrigger = new Trigger(this::isStalled)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the climber at the start of the autonomous phase.
   */
  public void autonomousInit() {
    resetEncoder();
    Utils.logInfo("Climber subsystem initialized for autonomous");
  }

  /**
   * Initializes the climber at the start of the teleop phase.
   */
  public void teleopInit() {
    Utils.logInfo("Climber subsystem initialized for teleop");
  }

  /**
   * Initializes the climber for post match (disabled) state.
   */
  public void postMatch() {
    Utils.logInfo("Climber subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

  /**
   * Command to move the climber to a specific position
   * @param targetDegrees Target position in degrees
   * @param atTarget BooleanSupplier that returns true when the climber is at the target
   * @return Command that moves the climber to the target position
   */
  public Command toPositionCommand(double targetDegrees, BooleanSupplier atTarget) {
    return startEnd(
      () -> setPosition(targetDegrees),
      () -> {}
    )
    .until(atTarget)
    .withTimeout(ClimberConstants.kMoveTimeoutSeconds)
    .finallyDo(this::stop)
    .withName("Climber_MoveToPosition");
  }

  /**
   * Command to move the climber to the home position
   * @return Command that moves the climber to the home position
   */
  public Command toHomeCommand() {
    return toPositionCommand(ClimberConstants.kHomeDegrees, this::isAtHomePosition)
      .withName("Climber_Home");
  }

  /**
   * Command to move the climber to the level one position
   * @return Command that moves the climber to the level one position
   */
  public Command toLevelOneCommand() {
    return toPositionCommand(ClimberConstants.kLevelOneClimbDegrees, this::isAtLevelOneClimbPosition)
      .withName("Climber_LevelOne");
  }

  /**
   * Command to move the climber to the level two position
   * @return Command that moves the climber to the level two position
   */
  public Command toLevelTwoCommand() {
    return toPositionCommand(ClimberConstants.kLevelTwoClimbDegrees, this::isAtLevelTwoClimbPosition)
      .withName("Climber_LevelTwo");
  }
  
  /**
   * Command the climber to the upper limit
   * @return Command that rotates to upper limit then stops
   */
  public Command toUpperLimitCommand() {
    return toPositionCommand(ClimberConstants.kUpperLimitDegrees, this::isAtUpperLimit)
      .withName("Climber_UpToLimit");
  }
  
  /**
   * Command the climber to the lower limit
   * @return Command that rotates to lower limit then stops
   */
  public Command toLowerLimitCommand() {
    return toPositionCommand(ClimberConstants.kLowerLimitDegrees, this::isAtLowerLimit)
      .withName("Climber_DownToLimit");
  }

  /**
   * Command to stop the climber
   * @return Command that stops the climber motor
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Climber_Stop");
  }

  /**
   * Command to move the climber up (open-loop control)
   * @return Command that moves the climber up
   */
  public Command upCommand() {
    return run(() -> setVoltage(ClimberConstants.kManualUpVoltage))
      .withName("Climber_ManualUp");
  }

  /**
   * Command to move the climber up (open-loop control)
   * @return Command that moves the climber up
   */
  public Command downCommand() {
    return run(() -> setVoltage(ClimberConstants.kManualDownVoltage))
      .withName("Climber_ManualDown");
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
    builder.addDoubleProperty("Target Angle (deg)",  () -> Utils.showDouble(targetAngleDegrees), null);
    builder.addDoubleProperty("Current Angle (deg)", () -> Utils.showDouble(getPosition()), null);
    builder.addDoubleProperty("Current (A)",         () -> Utils.showDouble(climberMotor.getOutputCurrent()), null);
    builder.addDoubleProperty("Temp (C)",            () -> Utils.showDouble(climberMotor.getMotorTemperature()), null);
  }
}
