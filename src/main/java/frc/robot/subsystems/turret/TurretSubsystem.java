// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Conversions;
import frc.robot.util.TalonFXFactory;
import frc.robot.util.Utils;

/**
 * Turret rotation subsystem using a Falcon 500 (TalonFX). Rotation only --
 * the flywheel shooter mounted on the turret is a separate ShooterSubsystem
 * so the two can be commanded and scheduled independently.
 *
 * NOTE: This subsystem assumes the turret has a LIMITED mechanical range of
 * motion (no slip rings) bounded by TurretConstants.kMinAngleDegrees and
 * kMaxAngleDegrees. setTurretAngle() will refuse to wind past that range even
 * if the shortest path to a target would require it.
 *
 * Turret tuning process:
 * 1. Run turret SysId to characterize kS, kV, kA
 * 2. Tune MotionMagic cruise velocity and acceleration
 * 3. Tune kP until fast response without overshoot
 */
public class TurretSubsystem extends SubsystemBase {
  // Turret hardware
  private final TalonFX turretMotor;

  // Turret control request
  private final MotionMagicVoltage motionMagicRequest;

  // Cached target (for telemetry)
  private double targetAngleDegrees = 0.0;

  // SysId routine
  private final SysIdRoutine turretSysIdRoutine;

  /**
   * Creates a new TurretSubsystem
   */
  public TurretSubsystem() {
    // Create the turret motor with the given configuration
    turretMotor = TalonFXFactory.createMotor(CANConstants.kTurretMotorID, getMotorConfig());

    // Initialize control request
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Zero turret encoder at startup - turret must be at home position
    resetEncoder();

    // Initialize SysId routine
    turretSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> turretMotor.setControl(new VoltageOut(volts.in(Volts))),
        log -> log.motor("turret")
          .voltage(Volts.of(turretMotor.getMotorVoltage().getValueAsDouble()))
          .angularPosition(Rotations.of(turretMotor.getPosition().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(turretMotor.getVelocity().getValueAsDouble())),
        this
      )
    );

    // Set default command
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Turret", this);

    // Output initialization progress
    Utils.logInfo("Turret subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles the control loop onboard
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
      .withSupplyCurrentLimit(30)
      .withSupplyCurrentLowerLimit(20)
      .withSupplyCurrentLowerTime(1.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(40);

    // Set voltage limits
    config.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Set PID and feedforward gains
    config.Slot0
      .withKP(TurretConstants.kP)
      .withKI(TurretConstants.kI)
      .withKD(TurretConstants.kD)
      .withKS(TurretConstants.kS)
      .withKV(TurretConstants.kV)
      .withKA(TurretConstants.kA);

    // Set motion control parameters
    config.MotionMagic
      .withMotionMagicCruiseVelocity(TurretConstants.kCruiseVelocity)
      .withMotionMagicAcceleration(TurretConstants.kAcceleration)
      .withMotionMagicJerk(TurretConstants.kJerk);

    // Set soft limits for the elevator
    config.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(Conversions.degreesToRotations(
        TurretConstants.kMaxAngleDegrees,
        TurretConstants.kTurretGearRatio
      ))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(Conversions.degreesToRotations(
        TurretConstants.kMinAngleDegrees,
        TurretConstants.kTurretGearRatio
      ));

    // Keep false: ContinuousWrap on motor rotations would ruin turret positioning 
    config.ClosedLoopGeneral.ContinuousWrap = false;

    // Return the motor configuration
    return config;
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the turret to a target angle using MotionMagic ensuring the target
   * angle is within the safe range defined by TurretConstants.
   * @param angleDegrees Target angle in degrees
   */
  private void setTurretAngle(double angleDegrees) {
    // Clamp target to valid range
    targetAngleDegrees = MathUtil.clamp(
      Utils.normalizeAngleDegrees(angleDegrees), 
      TurretConstants.kMinAngleDegrees, 
      TurretConstants.kMaxAngleDegrees
    );

    // Convert target angle to motor rotations
    double targetMotorRotations = Conversions.degreesToRotations(
      targetAngleDegrees, 
      TurretConstants.kTurretGearRatio
    );

    // Set the target position using MotionMagic
    turretMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
  }

  /**
   * Sets the turret motor voltage directly (open-loop), with safety checks for limits.
   * @param volts Voltage to apply to the motor
   */
  private void setVoltage(double volts) {
    // Clamp voltage to safe range
    volts = MathUtil.clamp(volts, -12, 12);

    // Check if the turret is at the upper limit and trying to move up
    if (isAtUpperLimit() && volts > 0) {
      stop();
      return;
    }

    // Check if the turret is at the lower limit and trying to move down
    if (isAtLowerLimit() && volts < 0) {
      stop();
      return;
    }

    // Set the voltage to the turret motor
    turretMotor.setControl(new VoltageOut(volts));
  }

  /**
   * Gets the current turret angle in degrees, normalized to (-180, 180]
   * @return Current angle in degrees
   */
  private double getAngleDegrees() {
    return Utils.normalizeAngleDegrees(Conversions.rotationsToDegrees(
      turretMotor.getPosition().getValueAsDouble(), 
      TurretConstants.kTurretGearRatio
    ));
  }

  /**
   * Zeros the turret encoder. Turret must be at home position when called.
   */
  private void resetEncoder() {
    turretMotor.setPosition(0);
  }

  /**
   * Stops the turret motor
   */
  private void stop() {
    turretMotor.stopMotor();
  }

  /**
   * Gets whether the turret is at its target angle within tolerance
   * @return True if at target
   */
  private boolean isAtAngle(double targetAngleDegrees) {
    return MathUtil.isNear(
      Utils.normalizeAngleDegrees(targetAngleDegrees),
      getAngleDegrees(),
      TurretConstants.kAngleToleranceDegrees
    );
  }

  /**
   * Gets whether the turret is at its target angle within tolerance
   * @return True if at target
   */
  private boolean isAtAngle() {
    return isAtAngle(targetAngleDegrees);
  }
  
  /**
   * Check if turret is at or above upper position limit
   * @return true if at or past upper limit
   */
  private boolean isAtUpperLimit() {
    return isAtAngle(TurretConstants.kMaxAngleDegrees);
  }
  
  /**
   * Check if turret is at or below lower position limit
   * @return true if at or past lower limit
   */
  private boolean isAtLowerLimit() {
    return isAtAngle(TurretConstants.kMinAngleDegrees);
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isAtAngleTrigger = new Trigger(this::isAtAngle)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtUpperLimitTrigger = new Trigger(this::isAtUpperLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLowerLimitTrigger = new Trigger(this::isAtLowerLimit)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the turret at the start of the autonomous phase.
   */
  public void autonomousInit() {
    Utils.logInfo("Turret subsystem initialized for autonomous");
  }

  /**
   * Initializes the turret at the start of the teleop phase.
   */
  public void teleopInit() {
    Utils.logInfo("Turret subsystem initialized for teleop");
  }

  /**
   * Initializes the turret for post match (disabled) state.
   */
  public void postMatch() {
    Utils.logInfo("Turret subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return turretSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return turretSysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

  /**
   * Command to rotate the turret to a robot-relative angle.
   * @param angleDegrees The desired robot-relative angle in degrees
   * @return Command to rotate to the given angle
   */
  public Command setTurretAngleCommand(double angleDegrees) {
    return startEnd(
      () -> setTurretAngle(angleDegrees),
      () -> {}
    )
    .until(this::isAtAngle)
    .withTimeout(TurretConstants.kMoveTimeoutSeconds)
    .finallyDo(this::stop)
    .withName("Turret_SetTurretAngle");
  }

  /**
   * Command to continuously rotate the turret to track a field-relative angle,
   * accounting for the robot's current heading.
   * @param fieldAngleDegreesSupplier Supplier for the desired field-relative angle
   * @param robotHeadingDegreesSupplier Supplier for the robot's current field heading
   * @return Command to continuously track the field-relative angle
   */
  public Command aimAtFieldAngleCommand(
    DoubleSupplier fieldAngleDegreesSupplier,
    DoubleSupplier robotHeadingDegreesSupplier
  ) {
    return run(() -> {
      double turretAngle = Utils.normalizeAngleDegrees(
        fieldAngleDegreesSupplier.getAsDouble() - robotHeadingDegreesSupplier.getAsDouble()
      );
      setTurretAngle(turretAngle);
    })
    .withName("Turret_AimAtFieldAngle");
  }

  /**
   * Command to continuously rotate the turret to track a field-relative angle,
   * accounting for the robot's current heading.
   * @param fieldAngleDegreesSupplier Supplier for the desired field-relative angle
   * @param robotPoseSupplier Supplier for the robot's current field pose
   * @return Command to continuously track the field-relative angle
   */
  public Command aimAtFieldAngleCommand(
    DoubleSupplier fieldAngleDegreesSupplier,
    Supplier<Pose2d> robotPoseSupplier
  ) {
    return aimAtFieldAngleCommand(
      fieldAngleDegreesSupplier,
      () -> robotPoseSupplier.get().getRotation().getDegrees()
    );
  }

  /**
   * Command to continuously rotate the turret to face a target pose on the field.
   * @param robotPoseSupplier Supplier for the robot's current field pose
   * @param targetPoseSupplier Supplier for the field-relative target pose to face
   * @return Command to continuously aim at the target pose
   */
  public Command aimAtPoseCommand(
    Supplier<Pose2d> robotPoseSupplier,
    Supplier<Pose2d> targetPoseSupplier
  ) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier == null ? null : robotPoseSupplier.get();
      Pose2d targetPose = targetPoseSupplier == null ? null : targetPoseSupplier.get();

      // If either pose is null, we can't calculate the angle, so just return early
      if (robotPose == null || targetPose == null) {
        stop();
        return;
      }

      // Calculate the angle to the target pose in field coordinates
      double dx = targetPose.getX() - robotPose.getX();
      double dy = targetPose.getY() - robotPose.getY();
      double fieldAngleDegrees = Units.radiansToDegrees(Math.atan2(dy, dx));
      double turretAngle = Utils.normalizeAngleDegrees(fieldAngleDegrees - robotPose.getRotation().getDegrees());
      
      // Command the turret to the calculated angle
      setTurretAngle(turretAngle);
    })
    .withName("Turret_AimAtPose");
  }

  /**
   * Command to stop the turret.
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Turret_Stop");
  }

  /**
   * Command to move the turret clockwise (open-loop control)
   * @return Command that moves the turret clockwise
   */
  public Command clockwiseCommand() {
    return run(() -> setVoltage(TurretConstants.kManualClockwiseVoltage))
      .withName("Turret_ManualClockwise");
  }

  /**
   * Command to move the turret counter-clockwise (open-loop control)
   * @return Command that moves the turret counter-clockwise
   */
  public Command counterClockwiseCommand() {
    return run(() -> setVoltage(TurretConstants.kManualCounterClockwiseVoltage))
      .withName("Turret_ManualCounterClockwise");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Angle (deg)",  () -> Utils.showDouble(targetAngleDegrees), null);
    builder.addDoubleProperty("Current Angle (deg)", () -> Utils.showDouble(getAngleDegrees()), null);
    builder.addDoubleProperty("Current (A)",         () -> Utils.showDouble(turretMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Temp (C)",            () -> Utils.showDouble(turretMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
