// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.util.Conversions;
import frc.robot.util.TalonFXFactory.MotorPair;
import frc.robot.util.TalonFXFactory;
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
    // Create leader and follower motors
    MotorPair motors = TalonFXFactory.createMotorPair(
      CANConstants.kElevatorLeaderMotorID,
      CANConstants.kElevatorFollowerMotorID,
      getMotorConfig(),
      false
    );
    leaderMotor = motors.leader();
    followerMotor = motors.follower();

    // Initialize control requests
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Zero encoder at startup - elevator must be at home position
    resetEncoder();

    // Initialize SysId routine (leader motor only)
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> leaderMotor.setControl(new VoltageOut(volts.in(Volts))),
        log -> log.motor("leader")
          .voltage(Volts.of(leaderMotor.getMotorVoltage().getValueAsDouble()))
          .angularPosition(Rotations.of(leaderMotor.getPosition().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(leaderMotor.getVelocity().getValueAsDouble())),
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
  
  /**
   * Configure the subsystem motors with the appropriate settings
   */
  private TalonFXConfiguration getMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set basic motor configuration parameters
    config.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    // Set current limits
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)
      .withSupplyCurrentLowerLimit(30)
      .withSupplyCurrentLowerTime(1.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(60);

    // Set voltage limits
    config.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Set PID and feedforward gains
    config.Slot0
      .withKP(ElevatorConstants.kP)
      .withKI(ElevatorConstants.kI)
      .withKD(ElevatorConstants.kD)
      .withKS(ElevatorConstants.kS)
      .withKV(ElevatorConstants.kV)
      .withKA(ElevatorConstants.kA)
      .withKG(ElevatorConstants.kG)
      .withGravityType(GravityTypeValue.Elevator_Static);

    // Set motion control parameters
    config.MotionMagic
      .withMotionMagicCruiseVelocity(ElevatorConstants.kCruiseVelocity)
      .withMotionMagicAcceleration(ElevatorConstants.kAcceleration)
      .withMotionMagicJerk(ElevatorConstants.kJerk);

    // Set soft limits for the elevator
    config.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(Conversions.metersToRotations(
        ElevatorConstants.kMaxHeightMeters, 
        ElevatorConstants.kGearRatio, 
        ElevatorConstants.kSpoolCircumferenceMeters
      ))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(Conversions.metersToRotations(
        ElevatorConstants.kMinHeightMeters, 
        ElevatorConstants.kGearRatio, 
        ElevatorConstants.kSpoolCircumferenceMeters
      ));

    // Return the motor configuration
    return config;
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

    // Convert target height to motor rotations
    double targetMotorRotations = Conversions.metersToRotations(
      targetHeightMeters, 
      ElevatorConstants.kGearRatio, 
      ElevatorConstants.kSpoolCircumferenceMeters
    );

    // Set the target position using MotionMagic
    leaderMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
  }

  /**
   * Sets the elevator motor voltage directly (open-loop), with safety checks for limits.
   * @param volts Voltage to apply to the motor
   */
  private void setVoltage(double volts) {
    // Clamp voltage to safe range
    volts = MathUtil.clamp(volts, -12, 12);

    // Check if the elevator is at the upper limit and trying to move up
    if (isAtHeight(ElevatorConstants.kMaxHeightMeters) && volts > 0) {
      stop();
      return;
    }

    // Check if the elevator is at the lower limit and trying to move down
    if (isAtHeight(ElevatorConstants.kMinHeightMeters) && volts < 0) {
      stop();
      return;
    }

    // Set the voltage to the leader motor (follower will mirror)
    leaderMotor.setControl(new VoltageOut(volts));
  }

  /**
   * Gets the current elevator height in meters
   * @return Current height in meters
   */
  private double getHeightMeters() {
    return Conversions.rotationsToMeters(
      leaderMotor.getPosition().getValueAsDouble(),
      ElevatorConstants.kGearRatio,
      ElevatorConstants.kSpoolCircumferenceMeters
    );
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

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------
  
  public final Trigger isAtMinHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kMinHeightMeters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtMaxHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kMaxHeightMeters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelOneHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kHeightL1Meters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelTwoHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kHeightL2Meters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelThreeHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kHeightL3Meters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLevelFourHeightTrigger = new Trigger(() -> isAtHeight(ElevatorConstants.kHeightL4Meters))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the elevator at the start of the autonomous phase.
   */
  public void autonomousInit() {
    Utils.logInfo("Elevator subsystem initialized for autonomous");
  }

  /**
   * Initializes the elevator at the start of the teleop phase.
   */
  public void teleopInit() {
    Utils.logInfo("Elevator subsystem initialized for teleop");
  }

  /**
   * Initializes the elevator for post match (disabled) state.
   */
  public void postMatch() {
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
  public Command toHeightCommand(double heightMeters) {
    return startEnd(
      () -> setHeight(heightMeters),
      () -> {}
    )
    .until(() -> isAtHeight(heightMeters))
    .withTimeout(ElevatorConstants.kMoveTimeoutSeconds)
    .finallyDo(this::stop)
    .withName("Elevator_MoveToHeight");
  }

  /**
   * Command to move the elevator to the bottom and wait until it arrives.
   * @return Command to move to the bottom
   */
  public Command toMinimumCommand() {
    return toHeightCommand(ElevatorConstants.kMinHeightMeters)
      .withName("Elevator_MoveToMinimum");
  }

  /**
   * Command to move the elevator to the level one height in meters and wait until it arrives.
   * @return Command to move to the level one height
   */
  public Command toLevelOneCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL1Meters)
      .withName("Elevator_MoveToLevelOne");
  }

  /**
   * Command to move the elevator to the level two height in meters and wait until it arrives.
   * @return Command to move to the level two height
   */
  public Command toLevelTwoCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL2Meters)
      .withName("Elevator_MoveToLevelTwo");
  }

  /**
   * Command to move the elevator to the level three height in meters and wait until it arrives.
   * @return Command to move to the level three height
   */
  public Command toLevelThreeCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL3Meters)
      .withName("Elevator_MoveToLevelThree");
  }

  /**
   * Command to move the elevator to the level four height in meters and wait until it arrives.
   * @return Command to move to the level four height
   */
  public Command toLevelFourCommand() {
    return toHeightCommand(ElevatorConstants.kHeightL4Meters)
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
   * Command to move the elevator up (open-loop control)
   * @return Command that moves the elevator up
   */
  public Command upCommand() {
    return run(() -> setVoltage(ElevatorConstants.kManualUpVoltage))
      .withName("Elevator_ManualUp");
  }

  /**
   * Command to move the climber up (open-loop control)
   * @return Command that moves the climber up
   */
  public Command downCommand() {
    return run(() -> setVoltage(ElevatorConstants.kManualDownVoltage))
      .withName("Elevator_ManualDown");
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
