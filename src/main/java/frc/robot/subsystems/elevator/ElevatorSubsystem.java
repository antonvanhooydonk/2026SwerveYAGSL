// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final TalonFXConfiguration leaderConfig;

  // Control requests
  private final MotionMagicVoltage motionMagicRequest;
  private final VoltageOut voltageRequest;

  // Target position (for telemetry)
  private double targetPositionMeters = 0.0;

  // Whether the elevator is homing
  private boolean homing = false;

  // SysId routine
  private final SysIdRoutine sysIdRoutine;

  /**
   * Creates a new ElevatorSubsystem
   */
  public ElevatorSubsystem() {
    // Initialize hardware
    leaderMotor = new TalonFX(CANConstants.kElevatorLeaderMotorID);
    followerMotor = new TalonFX(CANConstants.kElevatorFollowerMotorID);
    leaderConfig = new TalonFXConfiguration();

    // Initialize control requests
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);

    // Configure motors
    configureMotors();

    // Zero encoder at startup - elevator must be at home position
    zeroEncoder();

    // Initialize SysId routine (leader motor only)
    sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,   // Default ramp rate (1 V/s)
        null,   // Default step voltage (7V)
        null,   // Default timeout (10s)
        state -> SignalLogger.writeString("elevator-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> leaderMotor.setControl(voltageRequest.withOutput(volts.in(Volts))),
        null,
        this
      )
    );

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
    leaderConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    // Current limits
    leaderConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60)
      .withSupplyCurrentLowerLimit(40)
      .withSupplyCurrentLowerTime(0.5)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);

    // Voltage compensation
    leaderConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Feedback - convert motor rotations to meters of elevator travel
    // rotationsToMeters = sprocket circumference / gear ratio
    leaderConfig.Feedback
      .withSensorToMechanismRatio(ElevatorConstants.kGearRatio / ElevatorConstants.kSpoolCircumferenceMeters);

    // Soft limits to protect the elevator without limit switches
    leaderConfig.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(ElevatorConstants.kMaxHeightMeters)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(ElevatorConstants.kMinHeightMeters);

    // Position PID with gravity compensation (slot 0)
    leaderConfig.Slot0
      .withKP(ElevatorConstants.kP)
      .withKI(ElevatorConstants.kI)
      .withKD(ElevatorConstants.kD)
      .withKS(ElevatorConstants.kS)
      .withKV(ElevatorConstants.kV)
      .withKA(ElevatorConstants.kA)
      .withKG(ElevatorConstants.kG)
      .withGravityType(GravityTypeValue.Elevator_Static); // Constant gravity compensation

    // MotionMagic configuration
    leaderConfig.MotionMagic
      .withMotionMagicCruiseVelocity(ElevatorConstants.kCruiseVelocityMPS)
      .withMotionMagicAcceleration(ElevatorConstants.kAccelerationMPS2)
      .withMotionMagicJerk(ElevatorConstants.kJerkMPS3);

    // Apply configuration to leader
    leaderMotor.getConfigurator().apply(leaderConfig);

    // Optimize CAN status frames on leader
    leaderMotor.getPosition().setUpdateFrequency(100.0);
    leaderMotor.getVelocity().setUpdateFrequency(100.0);
    leaderMotor.getMotorVoltage().setUpdateFrequency(50.0);
    leaderMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    leaderMotor.getTorqueCurrent().setUpdateFrequency(50.0);
    leaderMotor.getDeviceTemp().setUpdateFrequency(4.0);
    leaderMotor.optimizeBusUtilization();

    // Configure follower to mirror leader in opposite direction
    // OpposeMasterDirection=true if the follower is mechanically mirrored
    followerMotor.setControl(new Follower(CANConstants.kElevatorLeaderMotorID, MotorAlignmentValue.Aligned));

    // Minimize follower CAN traffic since it mirrors the leader
    followerMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    followerMotor.getDeviceTemp().setUpdateFrequency(4.0);
    followerMotor.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Zeros the encoder. Elevator must be at home (bottom) position when called.
   */
  private void zeroEncoder() {
    leaderMotor.setPosition(0);
  }

  /**
   * Gets the current elevator height in meters
   * @return Current height in meters
   */
  private double getHeightMeters() {
    return leaderMotor.getPosition().getValueAsDouble();
  }

  /**
   * Gets the current elevator velocity in meters per second
   * @return Current velocity in m/s
   */
  private double getVelocityMPS() {
    return leaderMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the elevator to a target height using MotionMagic
   * @param heightMeters Target height in meters
   */
  private void setHeight(double heightMeters) {
    // Clamp target to valid range
    targetPositionMeters = MathUtil.clamp(
      heightMeters,
      ElevatorConstants.kMinHeightMeters,
      ElevatorConstants.kMaxHeightMeters
    );

    leaderMotor.setControl(motionMagicRequest.withPosition(targetPositionMeters));
  }

  /**
   * Drives the elevator at a raw voltage. Used for homing.
   * @param volts Voltage to apply
   */
  private void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
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
   * @return True if at target
   */
  private boolean isAtTarget() {
    return Math.abs(targetPositionMeters - getHeightMeters()) < ElevatorConstants.kHeightToleranceMeters;
  }

  /**
   * Gets whether the elevator is at its minimum height (home position)
   * @return True if at home
   */
  private boolean isAtHome() {
    return getHeightMeters() <= ElevatorConstants.kMinHeightMeters + ElevatorConstants.kHeightToleranceMeters;
  }

  /**
   * Gets whether the elevator is at its maximum height
   * @return True if at max height
   */
  private boolean isAtMaxHeight() {
    return getHeightMeters() >= ElevatorConstants.kMaxHeightMeters - ElevatorConstants.kHeightToleranceMeters;
  }

  /**
   * Gets whether the elevator is currently homing
   * @return True if homing
   */
  private boolean isHoming() {
    return homing;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isAtTargetTrigger    = new Trigger(this::isAtTarget);
  public final Trigger isAtHomeTrigger      = new Trigger(this::isAtHome);
  public final Trigger isAtMaxHeightTrigger = new Trigger(this::isAtMaxHeight);
  public final Trigger isHomingTrigger      = new Trigger(this::isHoming);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the elevator at the start of the autonomous phase.
   */
  public void autonomousInit() {
    setMotorBrake(true);
    homing = false;
    setHeight(ElevatorConstants.kMinHeightMeters);
    Utils.logInfo("Elevator subsystem initialized for autonomous");
  }

  /**
   * Initializes the elevator at the start of the teleop phase.
   */
  public void teleopInit() {
    setMotorBrake(true);
    homing = false;
    Utils.logInfo("Elevator subsystem initialized for teleop");
  }

  /**
   * Initializes the elevator for post match (disabled) state.
   */
  public void postMatch() {
    setMotorBrake(false);
    homing = false;
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
  public Command moveToHeightCommand(double heightMeters) {
    return runOnce(() -> setHeight(heightMeters))
      .andThen(Commands.waitUntil(this::isAtTarget))
      .withName("Elevator_MoveToHeight");
  }

  /**
   * Command to move the elevator to its minimum (home) position and rezero the encoder.
   * Since there are no limit switches, homing uses a slow downward voltage until
   * the elevator stops moving, then zeros the encoder.
   * @return Command to home the elevator
   */
  public Command homeCommand() {
    return runOnce(() -> {
      homing = true;
      // Disable soft limits during homing so we can drive to the hard stop
      leaderMotor.getConfigurator().apply(
        leaderConfig.SoftwareLimitSwitch
          .withForwardSoftLimitEnable(false)
          .withReverseSoftLimitEnable(false)
      );
    })
    // Drive slowly downward
    .andThen(run(() -> setVoltage(ElevatorConstants.kHomingVoltage)))
    // Wait until velocity is near zero (hit the hard stop)
    .until(() -> Math.abs(getVelocityMPS()) < ElevatorConstants.kHomingVelocityThresholdMPS)
    // Zero the encoder and re-enable soft limits
    .andThen(runOnce(() -> {
      stop();
      zeroEncoder();
      targetPositionMeters = 0.0;
      homing = false;
      // Re-enable soft limits after homing
      leaderMotor.getConfigurator().apply(
        leaderConfig.SoftwareLimitSwitch
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(ElevatorConstants.kMaxHeightMeters)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(ElevatorConstants.kMinHeightMeters)
      );
    }))
    .withName("Elevator_Home");
  }

  /**
   * Command to stop the elevator
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("Elevator_Stop");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Height (m)",   () -> Utils.showDouble(targetPositionMeters), null);
    builder.addDoubleProperty("Current Height (m)",  () -> Utils.showDouble(getHeightMeters()), null);
    builder.addDoubleProperty("Height Error (m)",    () -> Utils.showDouble(targetPositionMeters - getHeightMeters()), null);
    builder.addDoubleProperty("Velocity (mps)",      () -> Utils.showDouble(getVelocityMPS()), null);
    builder.addBooleanProperty("At Target",          this::isAtTarget, null);
    builder.addBooleanProperty("At Home",            this::isAtHome, null);
    builder.addBooleanProperty("At Max Height",      this::isAtMaxHeight, null);
    builder.addBooleanProperty("Homing",             this::isHoming, null);
    builder.addDoubleProperty("Leader Voltage (V)",  () -> Utils.showDouble(leaderMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Current (A)",  () -> Utils.showDouble(leaderMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Temp (C)",     () -> Utils.showDouble(leaderMotor.getDeviceTemp().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Current (A)",() -> Utils.showDouble(followerMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Temp (C)",   () -> Utils.showDouble(followerMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
