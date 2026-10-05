// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
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
import frc.robot.util.Conversions;
import frc.robot.util.SparkMaxFactory;
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
    // Initialize the climber motor (we're using a brushed CIM for the climber)
    climberMotor = SparkMaxFactory.createBrushedMotor(CANConstants.kClimberMotorID, getMotorConfig());

    // Initialize closed-loop controller
    climberController = climberMotor.getClosedLoopController();
    
    // Initialize encoder
    climberEncoder = climberMotor.getEncoder();

    // Reset the encoder (assumes climber starts at the home position)
    resetEncoder();

    // Initialize SysId routine (leader motor only)
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> climberMotor.setVoltage(volts),
        log -> log.motor("climber")
          .voltage(Volts.of(climberMotor.getAppliedOutput() * climberMotor.getBusVoltage()))
          .angularPosition(Degrees.of(climberEncoder.getPosition()))
          .angularVelocity(DegreesPerSecond.of(climberEncoder.getVelocity())),
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

  @Override
  public void periodic() {}
  
  /**
   * Configure the subsystem motors with the appropriate settings
   */
  private SparkMaxConfig getMotorConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    
    // Set basic motor parameters
    config
      .smartCurrentLimit(30) 
      .voltageCompensation(12) 
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    // Set PID gains for closed-loop control
    config.closedLoop
      .p(ClimberConstants.kClimberKP)
      .i(ClimberConstants.kClimberKI)
      .d(ClimberConstants.kClimberKD);

    // Set motion control parameters for closed-loop control
    config.closedLoop.maxMotion
      .cruiseVelocity(Conversions.degreesToRotations(
        ClimberConstants.kMaxVelocityDegPerSec, 
        ClimberConstants.kGearRatio
      ) * 60)
      .maxAcceleration(Conversions.degreesToRotations(
        ClimberConstants.kMaxAccelDegPerSec2, 
        ClimberConstants.kGearRatio
      ) * 60)
      .allowedProfileError(Conversions.degreesToRotations(
        ClimberConstants.kAngleToleranceDegrees, 
        ClimberConstants.kGearRatio
      ));

    // Set soft limits to prevent over-rotation
    config.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(Conversions.degreesToRotations(
        ClimberConstants.kMaxAngleDegrees, 
        ClimberConstants.kGearRatio
      ))
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(Conversions.degreesToRotations(
        ClimberConstants.kMinAngleDegrees, 
        ClimberConstants.kGearRatio
      ));

    // Return the configured SparkMaxConfig
    return config;
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------
  
  /**
   * Set the target angle for the climber with safety limits
   * @param degrees Target angle in degrees
   */
  private void setAngle(double degrees) {
    // Clamp target to valid range
    targetAngleDegrees = MathUtil.clamp(
      degrees, 
      ClimberConstants.kMinAngleDegrees, 
      ClimberConstants.kMaxAngleDegrees
    );

    // Convert target height to motor rotations
    double targetMotorRotations = Conversions.degreesToRotations(
      targetAngleDegrees, 
      ClimberConstants.kGearRatio
    );

    // Set the target position using max motion control
    climberController.setSetpoint(targetMotorRotations, ControlType.kMAXMotionPositionControl);
  }

  /**
   * Set the voltage for the climber motor (open-loop control)
   * @param volts Voltage to apply to the motor (positive = toward larger encoder values)
   */
  private void setVoltage(double volts) {
    // Clamp voltage to safe range
    volts = MathUtil.clamp(volts, -12, 12);

    // Check if the climber is at the upper limit and trying to move up
    if (isAtUpperLimit() && volts < 0) {
      stop();
      return;
    }

    // Check if the climber is at the lower limit and trying to move down
    if (isAtLowerLimit() && volts > 0) {
      stop();
      return;
    }

    // Apply voltage to the motor
    climberMotor.setVoltage(volts);
  }
    
  /**
   * Get the current angle of the climber
   * @return Angle in degrees
   */
  private double getAngleDegrees() {
    return Conversions.rotationsToDegrees(climberEncoder.getPosition(), ClimberConstants.kGearRatio);
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
   * Check if climber is at a specific target position within tolerance
   * @param targetDegrees The target position in degrees to check against
   * @return true if within tolerance of the target position  
   */
  private boolean isAtAngle(double targetDegrees) {
    return MathUtil.isNear(
      Utils.normalizeAngleDegrees(targetDegrees),
      getAngleDegrees(),
      ClimberConstants.kAngleToleranceDegrees
    );
  }
  
  /**
   * Check if climber is at or above upper position limit
   * Upper limit is the minimum angle (more negative) in our coordinate system
   * @return true if at or past upper limit
   */
  private boolean isAtUpperLimit() {
    return isAtAngle(ClimberConstants.kMinAngleDegrees);
  }
  
  /**
   * Check if climber is at or below lower position limit
   * Lower limit is the maximum angle (more positive) in our coordinate system
   * @return true if at or past lower limit
   */
  private boolean isAtLowerLimit() {
    return isAtAngle(ClimberConstants.kMaxAngleDegrees);
  }
  
  /**
   * Check if climber is at home position
   * @return true if within tolerance of home position
   */
  private boolean isAtHomePosition() {
    return isAtAngle(ClimberConstants.kHomeDegrees);
  }
  
  /**
   * Check if climber is at level 1 climb position
   * @return true if within tolerance of level 1 climb position
   */
  private boolean isAtLevelOneClimbPosition() {
    return isAtAngle(ClimberConstants.kLevelOneClimbDegrees);
  }
  
  /**
   * Check if climber is at level 2 climb position
   * @return true if within tolerance of level 2 climb position
   */
  private boolean isAtLevelTwoClimbPosition() {
    return isAtAngle(ClimberConstants.kLevelTwoClimbDegrees);
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

  public final Trigger isAtHomeTrigger = new Trigger(this::isAtHomePosition)
    .debounce(0.1, Debouncer.DebounceType.kRising);

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
  public Command setAngleCommand(double targetDegrees, BooleanSupplier atTarget) {
    return startEnd(
      () -> setAngle(targetDegrees),
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
    return setAngleCommand(ClimberConstants.kHomeDegrees, this::isAtHomePosition)
      .withName("Climber_Home");
  }

  /**
   * Command to move the climber to the level one position
   * @return Command that moves the climber to the level one position
   */
  public Command toLevelOneCommand() {
    return setAngleCommand(ClimberConstants.kLevelOneClimbDegrees, this::isAtLevelOneClimbPosition)
      .withName("Climber_LevelOne");
  }

  /**
   * Command to move the climber to the level two position
   * @return Command that moves the climber to the level two position
   */
  public Command toLevelTwoCommand() {
    return setAngleCommand(ClimberConstants.kLevelTwoClimbDegrees, this::isAtLevelTwoClimbPosition)
      .withName("Climber_LevelTwo");
  }
  
  /**
   * Command the climber to the upper limit
   * @return Command that rotates to upper limit then stops
   */
  public Command toUpperLimitCommand() {
    return setAngleCommand(ClimberConstants.kMinAngleDegrees, this::isAtUpperLimit)
      .withName("Climber_UpToLimit");
  }
  
  /**
   * Command the climber to the lower limit
   * @return Command that rotates to lower limit then stops
   */
  public Command toLowerLimitCommand() {
    return setAngleCommand(ClimberConstants.kMaxAngleDegrees, this::isAtLowerLimit)
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
    builder.addDoubleProperty("Current Angle (deg)", () -> Utils.showDouble(getAngleDegrees()), null);
    builder.addDoubleProperty("Current (A)",         () -> Utils.showDouble(climberMotor.getOutputCurrent()), null);
    builder.addDoubleProperty("Temp (C)",            () -> Utils.showDouble(climberMotor.getMotorTemperature()), null);
  }
}
