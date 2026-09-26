// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

/**
 * Elevator subsystem using dual Kraken X60 (TalonFX) motors with a follower configuration.
 * Uses MotionMagic with gravity compensation for smooth position control.
 *
 * Tuning process:
 * 1. Run SysId to characterize kS, kV, kA
 * 2. Tune kG until elevator holds position with no movement when commanded to hold
 * 3. Tune MotionMagic cruise velocity and acceleration
 * 4. Tune kP until fast response without overshoot
 *
 * IMPORTANT: Since there are no limit switches, the elevator must be at the
 * bottom (home) position when the robot code starts, as the encoder is zeroed
 * at construction. Call homeCommand() to return to home and rezero if needed.
 */
public class ElevatorSubsystem extends SubsystemBase {
  // Hardware - leader and follower motors
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final TalonFXConfiguration motorConfig;

  // Control requests
  private final MotionMagicVoltage motionMagicRequest;

  // Target position (for telemetry)
  private double targetHeightMeters = 0.0;

  // SysId routine
  private final SysIdRoutine sysIdRoutine;

  /**
   * Creates a new ElevatorSubsystem
   */
  public ElevatorSubsystem() {
    // Initialize hardware
    leaderMotor = new TalonFX(CANConstants.kElevatorLeaderMotorID);
    followerMotor = new TalonFX(CANConstants.kElevatorFollowerMotorID);
    motorConfig = new TalonFXConfiguration();

    // Initialize control requests
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Configure motors
    configureMotors();

    // Zero encoder at startup - elevator must be at home position
    resetEncoder();

    // Initialize SysId routine (leader motor only)
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("elevator-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> leaderMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );

    // Set default command
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Elevator", this);

    // Output initialization progress
    Utils.logInfo("Elevator subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles control loop onboard
  }

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

  /**
   * Configures both motors. The follower mirrors the leader in the opposite direction.
   */
  private void configureMotors() {
    // Motor output
    motorConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    // Current limits
    motorConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60)
      .withSupplyCurrentLowerLimit(40)
      .withSupplyCurrentLowerTime(0.5)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);

    // Voltage compensation
    motorConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Feedback - convert motor rotations to meters of elevator travel
    // rotationsToMeters = sprocket circumference / gear ratio
    motorConfig.Feedback
      .withSensorToMechanismRatio(ElevatorConstants.kGearRatio / ElevatorConstants.kSpoolCircumferenceMeters);

    // Soft limits to protect the elevator without limit switches
    motorConfig.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(ElevatorConstants.kMaxHeightMeters)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(ElevatorConstants.kMinHeightMeters);

    // Position PID with gravity compensation (slot 0)
    motorConfig.Slot0
      .withKP(ElevatorConstants.kP)
      .withKI(ElevatorConstants.kI)
      .withKD(ElevatorConstants.kD)
      .withKS(ElevatorConstants.kS)
      .withKV(ElevatorConstants.kV)
      .withKA(ElevatorConstants.kA)
      .withKG(ElevatorConstants.kG)
      .withGravityType(GravityTypeValue.Elevator_Static); // Constant gravity compensation

    // MotionMagic configuration
    motorConfig.MotionMagic
      .withMotionMagicCruiseVelocity(ElevatorConstants.kCruiseVelocityMPS)
      .withMotionMagicAcceleration(ElevatorConstants.kAccelerationMPS2)
      .withMotionMagicJerk(ElevatorConstants.kJerkMPS3);

    // Apply configuration to leader
    leaderMotor.getConfigurator().apply(motorConfig);
    followerMotor.getConfigurator().apply(motorConfig);

    // Configure follower to be aligned with the leader
    followerMotor.setControl(new Follower(CANConstants.kElevatorLeaderMotorID, MotorAlignmentValue.Aligned));

    // Optimize CAN status frames on leader
    leaderMotor.getPosition().setUpdateFrequency(100.0);
    leaderMotor.getVelocity().setUpdateFrequency(100.0);
    leaderMotor.getMotorVoltage().setUpdateFrequency(50.0);
    leaderMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    leaderMotor.getTorqueCurrent().setUpdateFrequency(50.0);
    leaderMotor.getDeviceTemp().setUpdateFrequency(4.0);
    leaderMotor.optimizeBusUtilization();

    // Minimize follower CAN traffic since it mirrors the leader
    followerMotor.getPosition().setUpdateFrequency(100.0);
    followerMotor.getVelocity().setUpdateFrequency(100.0);
    followerMotor.getMotorVoltage().setUpdateFrequency(50.0);
    followerMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    followerMotor.getTorqueCurrent().setUpdateFrequency(50.0);
    followerMotor.getDeviceTemp().setUpdateFrequency(4.0);
    followerMotor.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the elevator to a target height using MotionMagic
   * @param heightMeters Target height in meters
   */
  private void setHeight(double heightMeters) {
    // Clamp target to valid range
    targetHeightMeters = MathUtil.clamp(
      heightMeters,
      ElevatorConstants.kMinHeightMeters,
      ElevatorConstants.kMaxHeightMeters
    );

    // Set the target position using MotionMagic with gravity compensation
    leaderMotor.setControl(motionMagicRequest.withPosition(targetHeightMeters));
  }

  /**
   * Gets the current elevator height in meters
   * @return Current height in meters
   */
  private double getHeightMeters() {
    return leaderMotor.getPosition().getValueAsDouble();
  }
  
  /**
   * Reset the encoder position to zero.
   * This should only be called when the elevator is physically in the "home" position.
   */
  private void resetEncoder() {
    leaderMotor.setPosition(0);
  }

  /**
   * Stops both motors
   */
  private void stop() {
    leaderMotor.stopMotor();
  }

  /**
   * Sets motors to brake or coast mode
   * @param brake True for brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leaderMotor.setNeutralMode(mode);
    followerMotor.setNeutralMode(mode);
  }

  /**
   * Gets whether the elevator is at its target height within tolerance
   * @param targetHeightMeters Target height in meters
   * @return True if at target
   */
  private boolean isAtHeight(double targetHeightMeters) {
    return MathUtil.isNear(
      targetHeightMeters,
      getHeightMeters(),
      ElevatorConstants.kHeightToleranceMeters
    );
  }

  /**
   * Gets whether the elevator is at its minimum height (home position)
   * @return True if at home
   */
  private boolean isAtHomeHeight() {
    return isAtHeight(ElevatorConstants.kMinHeightMeters);
  }

  /**
   * Gets whether the elevator is at its maximum height
   * @return True if at max height
   */
  private boolean isAtMaxHeight() {
    return isAtHeight(ElevatorConstants.kMaxHeightMeters);
  }

  /**
   * Gets whether the elevator is at its minimum height
   * @return True if at min height
   */
  private boolean isAtMinHeight() {
    return isAtHeight(ElevatorConstants.kMinHeightMeters);
  }

  /**
   * Gets whether the elevator is at its level 1 height
   * @return True if at level 1 height
   */
  private boolean isAtLevelOneHeight() {
    return isAtHeight(ElevatorConstants.kHeightL1Meters);
  }

  /**
   * Gets whether the elevator is at its level 2 height
   * @return True if at level 2 height
   */
  private boolean isAtLevelTwoHeight() {
    return isAtHeight(ElevatorConstants.kHeightL2Meters);
  }

  /**
   * Gets whether the elevator is at level 3 height
   * @return True if at level 3 height
   */
  private boolean isAtLevelThreeHeight() {
    return isAtHeight(ElevatorConstants.kHeightL3Meters);
  }

  /**
   * Gets whether the elevator is at level 4 height
   * @return True if at level 4 height
   */
  private boolean isAtLevelFourHeight() {
    return isAtHeight(ElevatorConstants.kHeightL4Meters);
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isAtHomeTrigger = new Trigger(this::isAtHomeHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);
  
  public final Trigger isAtMinHeightTrigger = new Trigger(this::isAtMinHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtMaxHeightTrigger = new Trigger(this::isAtMaxHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelOneHeightTrigger = new Trigger(this::isAtLevelOneHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelTwoHeightTrigger = new Trigger(this::isAtLevelTwoHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelThreeHeightTrigger = new Trigger(this::isAtLevelThreeHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelFourHeightTrigger = new Trigger(this::isAtLevelFourHeight)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the elevator at the start of the autonomous phase.
   */
  public void autonomousInit() {
    setMotorBrake(true);
    setHeight(ElevatorConstants.kMinHeightMeters);
    Utils.logInfo("Elevator subsystem initialized for autonomous");
  }

  /**
   * Initializes the elevator at the start of the teleop phase.
   */
  public void teleopInit() {
    setMotorBrake(true);
    Utils.logInfo("Elevator subsystem initialized for teleop");
  }

  /**
   * Initializes the elevator for post match (disabled) state.
   */
  public void postMatch() {
    setMotorBrake(false);
    Utils.logInfo("Elevator subsystem initialized for post match");
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
   * Command to move the elevator to a target height in meters and wait until it arrives.
   * @param heightMeters Target height in meters
   * @return Command to move to the target height
   */
  public Command toHeightCommand(double heightMeters, BooleanSupplier atTarget) {
    return startEnd(
      () -> setHeight(heightMeters),
      () -> {}
    )
    .until(atTarget)
    .withTimeout(ElevatorConstants.kMoveTimeoutSeconds)
    .finallyDo(this::stop)
    .withName("Elevator_MoveToHeight");
  }

  /**
   * Command to move the elevator to the bottom height in meters and wait until it arrives.
   * @return Command to move to the lowest height
   */
  public Command toBottomCommand() {
    return toHeightCommand(ElevatorConstants.kMinHeightMeters, this::isAtMinHeight)
      .withName("Elevator_MoveToBottom");
  }

  /**
   * Command to move the elevator to the level one height in meters and wait until it arrives.
   * @return Command to move to the level one height
   */
  public Command toLevelOneCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL1Meters, this::isAtLevelOneHeight)
      .withName("Elevator_MoveToLevelOne");
  }

  /**
   * Command to move the elevator to the level two height in meters and wait until it arrives.
   * @return Command to move to the level two height
   */
  public Command toLevelTwoCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL2Meters, this::isAtLevelTwoHeight)
      .withName("Elevator_MoveToLevelTwo");
  }

  /**
   * Command to move the elevator to the level three height in meters and wait until it arrives.
   * @return Command to move to the level three height
   */
  public Command toLevelThreeCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL3Meters, this::isAtLevelThreeHeight)
      .withName("Elevator_MoveToLevelThree");
  }

  /**
   * Command to move the elevator to the level four height in meters and wait until it arrives.
   * @return Command to move to the level four height
   */
  public Command toLevelFourCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL4Meters, this::isAtLevelFourHeight)
      .withName("Elevator_MoveToLevelFour");
  }

  /**
   * Command to stop the elevator
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Elevator_Stop");
  }
  
  /**
   * Command to reset the encoder to zero at the current position
   * @return Command that resets the encoder
   */
  public Command setHomePositionCommand() {
    return runOnce(this::resetEncoder)
      .ignoringDisable(true)
      .withName("Elevator_SetHomePosition");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Height (m)",   () -> Utils.showDouble(targetHeightMeters), null);
    builder.addDoubleProperty("Current Height (m)",  () -> Utils.showDouble(getHeightMeters()), null);
    builder.addDoubleProperty("Leader Current (A)",  () -> Utils.showDouble(leaderMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Temp (C)",     () -> Utils.showDouble(leaderMotor.getDeviceTemp().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Current (A)",() -> Utils.showDouble(followerMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Temp (C)",   () -> Utils.showDouble(followerMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
