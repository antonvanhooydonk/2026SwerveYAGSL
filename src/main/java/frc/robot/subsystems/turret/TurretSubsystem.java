// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
 * Turret subsystem using a Falcon 500 (TalonFX) for rotation and dual
 * Kraken X60 (TalonFX) motors in a follower configuration for the flywheel.
 *
 * Turret tuning process:
 * 1. Run turret SysId to characterize kS, kV, kA
 * 2. Tune MotionMagic cruise velocity and acceleration
 * 3. Tune kP until fast response without overshoot
 *
 * Flywheel tuning process:
 * 1. Run flywheel SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class TurretSubsystem extends SubsystemBase {
  // Turret hardware
  private final TalonFX turretMotor;
  private final TalonFXConfiguration turretConfig;

  // Flywheel hardware - leader and follower
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;
  private final TalonFXConfiguration flywheelConfig;

  // Turret control request
  private final MotionMagicVoltage turretMotionMagicRequest;

  // Flywheel control requests
  private final VelocityVoltage flywheelVelocityRequest;

  // Cached targets (for telemetry)
  private double targetTurretAngleDegrees = 0.0;
  private double targetFlywheelRPM = 0.0;

  // SysId routines
  private final SysIdRoutine turretSysIdRoutine;
  private final SysIdRoutine flywheelSysIdRoutine;

  /**
   * Creates a new TurretSubsystem
   */
  public TurretSubsystem() {
    // Initialize turret hardware
    turretMotor = new TalonFX(CANConstants.kTurretMotorID);
    turretConfig = new TalonFXConfiguration();

    // Initialize flywheel hardware
    flywheelLeader = new TalonFX(CANConstants.kFlywheelLeaderMotorID);
    flywheelFollower = new TalonFX(CANConstants.kFlywheelFollowerMotorID);
    flywheelConfig = new TalonFXConfiguration();

    // Initialize control requests
    turretMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    // Configure motors
    configureTurretMotor();
    configureFlywheelMotors();

    // Zero turret encoder at startup - turret must be at home position
    zeroTurretEncoder();

    // Initialize SysId routines
    turretSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("turret-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> turretMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );

    flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("flywheel-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> flywheelLeader.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );

    // Add data to dashboard
    SmartDashboard.putData("Turret", this);

    // Output initialization progress
    Utils.logInfo("Turret subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles control loops onboard
  }

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

  /**
   * Configures the turret rotation motor
   */
  private void configureTurretMotor() {
    turretConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    turretConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)
      .withSupplyCurrentLowerLimit(30)
      .withSupplyCurrentLowerTime(0.5)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(60);

    turretConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    turretConfig.Slot0
      .withKP(TurretConstants.kTurretKP)
      .withKI(TurretConstants.kTurretKI)
      .withKD(TurretConstants.kTurretKD)
      .withKS(TurretConstants.kTurretKS)
      .withKV(TurretConstants.kTurretKV)
      .withKA(TurretConstants.kTurretKA);

    turretConfig.MotionMagic
      .withMotionMagicCruiseVelocity(TurretConstants.kTurretCruiseVelocityDPS)
      .withMotionMagicAcceleration(TurretConstants.kTurretAccelerationDPS2)
      .withMotionMagicJerk(TurretConstants.kTurretJerkDPS3);

    turretMotor.getConfigurator().apply(turretConfig);

    turretMotor.getPosition().setUpdateFrequency(100.0);
    turretMotor.getVelocity().setUpdateFrequency(100.0);
    turretMotor.getMotorVoltage().setUpdateFrequency(50.0);
    turretMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    turretMotor.getDeviceTemp().setUpdateFrequency(4.0);
    turretMotor.optimizeBusUtilization();
  }

  /**
   * Configures the flywheel leader and follower motors
   */
  private void configureFlywheelMotors() {
    flywheelConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast) // Coast so flywheel spins down naturally
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    flywheelConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60)
      .withSupplyCurrentLowerLimit(40)
      .withSupplyCurrentLowerTime(0.5)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);

    flywheelConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Velocity PID (slot 0) - velocity in RPS
    flywheelConfig.Slot0
      .withKP(TurretConstants.kFlywheelKP)
      .withKI(TurretConstants.kFlywheelKI)
      .withKD(TurretConstants.kFlywheelKD)
      .withKS(TurretConstants.kFlywheelKS)
      .withKV(TurretConstants.kFlywheelKV)
      .withKA(TurretConstants.kFlywheelKA);

    flywheelLeader.getConfigurator().apply(flywheelConfig);

    // Optimize CAN status frames on leader
    flywheelLeader.getVelocity().setUpdateFrequency(100.0);
    flywheelLeader.getMotorVoltage().setUpdateFrequency(50.0);
    flywheelLeader.getSupplyCurrent().setUpdateFrequency(50.0);
    flywheelLeader.getTorqueCurrent().setUpdateFrequency(50.0);
    flywheelLeader.getDeviceTemp().setUpdateFrequency(4.0);
    flywheelLeader.optimizeBusUtilization();

    // Configure follower to mirror leader
    flywheelFollower.setControl(new Follower(CANConstants.kFlywheelLeaderMotorID, MotorAlignmentValue.Opposed));

    // Minimize follower CAN traffic
    flywheelFollower.getSupplyCurrent().setUpdateFrequency(50.0);
    flywheelFollower.getDeviceTemp().setUpdateFrequency(4.0);
    flywheelFollower.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private turret state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Zeros the turret encoder. Turret must be at home position when called.
   */
  private void zeroTurretEncoder() {
    turretMotor.setPosition(0);
  }

  /**
   * Gets the current turret angle in degrees, normalized to (-180, 180]
   * @return Current angle in degrees
   */
  private double getTurretAngleDegrees() {
    return normalizeAngleDegrees(turretMotor.getPosition().getValueAsDouble());
  }

  /**
   * Sets the turret to a target angle using MotionMagic, taking the shortest path
   * @param angleDegrees Target angle in degrees
   */
  private void setTurretAngle(double angleDegrees) {
    double normalizedAngle = normalizeAngleDegrees(angleDegrees);

    // Find shortest path from current angle to target
    double currentAngle = getTurretAngleDegrees();
    double delta = normalizeAngleDegrees(normalizedAngle - currentAngle);

    // Add delta to raw motor position to preserve continuity
    double targetPosition = turretMotor.getPosition().getValueAsDouble() + delta;

    targetTurretAngleDegrees = normalizedAngle;
    turretMotor.setControl(turretMotionMagicRequest.withPosition(targetPosition));
  }

  /**
   * Stops the turret motor
   */
  private void stopTurret() {
    turretMotor.stopMotor();
  }

  /**
   * Gets whether the turret is at its target angle within tolerance
   * @return True if at target
   */
  private boolean isTurretAtTarget() {
    return Math.abs(normalizeAngleDegrees(targetTurretAngleDegrees - getTurretAngleDegrees()))
        < TurretConstants.kTurretAngleToleranceDegrees;
  }

  /**
   * Normalizes an angle to (-180, 180]
   * @param angleDegrees Angle in degrees
   * @return Normalized angle in degrees
   */
  private double normalizeAngleDegrees(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180.0, 180.0);
  }

  // ----------------------------------------------------------------------------------------
  // Private flywheel state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Gets the current flywheel velocity in RPM
   * @return Current velocity in RPM
   */
  private double getFlywheelRPM() {
    // TalonFX velocity is in RPS, convert to RPM
    return flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Sets the flywheel to a target velocity in RPM
   * @param rpm Target velocity in RPM
   */
  private void setFlywheelRPM(double rpm) {
    targetFlywheelRPM = rpm;
    // Convert RPM to RPS for TalonFX
    double rps = rpm / 60.0;
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(rps));
  }

  /**
   * Stops the flywheel
   */
  private void stopFlywheel() {
    targetFlywheelRPM = 0.0;
    flywheelLeader.stopMotor();
  }

  /**
   * Gets whether the flywheel is at its target velocity within tolerance
   * @return True if at target velocity
   */
  private boolean isFlywheelAtTarget() {
    // Don't report at target if flywheel is stopped
    if (targetFlywheelRPM == 0.0) {
      return false;
    }
    return Math.abs(targetFlywheelRPM - getFlywheelRPM()) < TurretConstants.kFlywheelToleranceRPM;
  }

  /**
   * Gets whether the flywheel is spinning (above a minimum threshold)
   * @return True if spinning
   */
  private boolean isFlywheelSpinning() {
    return getFlywheelRPM() > TurretConstants.kFlywheelMinSpinningRPM;
  }

  /**
   * Sets both motors to brake or coast mode
   * @param brake True for brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turretMotor.setNeutralMode(mode);

    // Flywheel always stays in coast mode
    flywheelLeader.setNeutralMode(NeutralModeValue.Coast);
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isTurretAtTargetTrigger    = new Trigger(this::isTurretAtTarget);
  public final Trigger isFlywheelAtTargetTrigger  = new Trigger(this::isFlywheelAtTarget);
  public final Trigger isFlywheelSpinningTrigger  = new Trigger(this::isFlywheelSpinning);

  /** True when both turret is aimed and flywheel is at speed - ready to shoot */
  public final Trigger isReadyToShootTrigger = isTurretAtTargetTrigger.and(isFlywheelAtTargetTrigger);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the turret at the start of the autonomous phase.
   */
  public void autonomousInit() {
    setMotorBrake(true);
    setTurretAngle(0);
    stopFlywheel();
    Utils.logInfo("Turret subsystem initialized for autonomous");
  }

  /**
   * Initializes the turret at the start of the teleop phase.
   */
  public void teleopInit() {
    setMotorBrake(true);
    stopFlywheel();
    Utils.logInfo("Turret subsystem initialized for teleop");
  }

  /**
   * Initializes the turret for post match (disabled) state.
   */
  public void postMatch() {
    setMotorBrake(false);
    stopFlywheel();
    Utils.logInfo("Turret subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command turretSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return turretSysIdRoutine.quasistatic(direction);
  }

  public Command turretSysIdDynamic(SysIdRoutine.Direction direction) {
    return turretSysIdRoutine.dynamic(direction);
  }

  public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheelSysIdRoutine.quasistatic(direction);
  }

  public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheelSysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

  /**
   * Command to rotate the turret to a robot-relative angle.
   * @param angleDegrees The desired robot-relative angle in degrees
   * @return Command to rotate to the given angle
   */
  public Command aimAtAngleCommand(double angleDegrees) {
    return runOnce(() -> setTurretAngle(angleDegrees));
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
      double turretAngle = normalizeAngleDegrees(
        fieldAngleDegreesSupplier.getAsDouble() - robotHeadingDegreesSupplier.getAsDouble()
      );
      setTurretAngle(turretAngle);
    });
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
      Pose2d robotPose = robotPoseSupplier.get();
      Pose2d targetPose = targetPoseSupplier.get();

      if (robotPose == null || targetPose == null) {
        return;
      }

      double dx = targetPose.getX() - robotPose.getX();
      double dy = targetPose.getY() - robotPose.getY();
      double fieldAngleDegrees = Units.radiansToDegrees(Math.atan2(dy, dx));
      double turretAngle = normalizeAngleDegrees(fieldAngleDegrees - robotPose.getRotation().getDegrees());
      setTurretAngle(turretAngle);
    });
  }

  /**
   * Command to spin the flywheel to a target velocity and wait until it is at speed.
   * @param rpm Target velocity in RPM
   * @return Command to spin up the flywheel
   */
  public Command spinUpCommand(double rpm) {
    return runOnce(() -> setFlywheelRPM(rpm))
      .andThen(Commands.waitUntil(this::isFlywheelAtTarget));
  }

  /**
   * Command to spin the flywheel to a target velocity without waiting.
   * Useful when pre-spinning during aiming.
   * @param rpm Target velocity in RPM
   * @return Command to set flywheel velocity
   */
  public Command setFlywheelRPMCommand(double rpm) {
    return runOnce(() -> setFlywheelRPM(rpm));
  }

  /**
   * Command to stop the flywheel.
   * @return Command to stop the flywheel
   */
  public Command stopFlywheelCommand() {
    return runOnce(this::stopFlywheel);
  }

  /**
   * Command to aim at a target pose and spin up the flywheel simultaneously,
   * then signal ready when both are on target.
   * This is the primary shoot preparation command.
   * @param robotPoseSupplier Supplier for the robot's current field pose
   * @param targetPoseSupplier Supplier for the field-relative target pose
   * @param rpm Target flywheel velocity in RPM
   * @return Command that aims and spins up in parallel
   */
  public Command prepareToShootCommand(
    Supplier<Pose2d> robotPoseSupplier,
    Supplier<Pose2d> targetPoseSupplier,
    double rpm
  ) {
    return Commands.parallel(
      aimAtPoseCommand(robotPoseSupplier, targetPoseSupplier),
      setFlywheelRPMCommand(rpm)
    );
  }

  /**
   * Command to home the turret to 0 degrees and stop the flywheel.
   * @return Command to home the turret
   */
  public Command homeCommand() {
    return runOnce(() -> {
      setTurretAngle(0);
      stopFlywheel();
    }).andThen(Commands.waitUntil(this::isTurretAtTarget));
  }

  /**
   * Command to stop both the turret and flywheel.
   */
  public Command stopCommand() {
    return runOnce(() -> {
      stopTurret();
      stopFlywheel();
    });
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    // Turret telemetry
    builder.addDoubleProperty("Turret Target Angle (deg)",  () -> Utils.showDouble(targetTurretAngleDegrees), null);
    builder.addDoubleProperty("Turret Current Angle (deg)", () -> Utils.showDouble(getTurretAngleDegrees()), null);
    builder.addDoubleProperty("Turret Angle Error (deg)",   () -> Utils.showDouble(normalizeAngleDegrees(targetTurretAngleDegrees - getTurretAngleDegrees())), null);
    builder.addBooleanProperty("Turret At Target",          this::isTurretAtTarget, null);
    builder.addDoubleProperty("Turret Voltage (V)",         () -> Utils.showDouble(turretMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Turret Current (A)",         () -> Utils.showDouble(turretMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Turret Temp (C)",            () -> Utils.showDouble(turretMotor.getDeviceTemp().getValueAsDouble()), null);

    // Flywheel telemetry
    builder.addDoubleProperty("Flywheel Target RPM",        () -> Utils.showDouble(targetFlywheelRPM), null);
    builder.addDoubleProperty("Flywheel Current RPM",       () -> Utils.showDouble(getFlywheelRPM()), null);
    builder.addDoubleProperty("Flywheel RPM Error",         () -> Utils.showDouble(targetFlywheelRPM - getFlywheelRPM()), null);
    builder.addBooleanProperty("Flywheel At Target",        this::isFlywheelAtTarget, null);
    builder.addBooleanProperty("Flywheel Spinning",         this::isFlywheelSpinning, null);
    builder.addBooleanProperty("Ready To Shoot",            () -> isReadyToShootTrigger.getAsBoolean(), null);
    builder.addDoubleProperty("Flywheel Voltage (V)",       () -> Utils.showDouble(flywheelLeader.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Flywheel Current (A)",       () -> Utils.showDouble(flywheelLeader.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Flywheel Temp (C)",          () -> Utils.showDouble(flywheelLeader.getDeviceTemp().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Current (A)",       () -> Utils.showDouble(flywheelFollower.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Temp (C)",          () -> Utils.showDouble(flywheelFollower.getDeviceTemp().getValueAsDouble()), null);
  }
}
