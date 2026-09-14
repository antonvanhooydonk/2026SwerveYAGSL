// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
 * Shooter flywheel subsystem using dual Kraken X60 (TalonFX) motors in a
 * follower configuration. Physically mounted on the turret, but kept as a
 * separate subsystem from TurretSubsystem so aiming and spin-up can be
 * commanded and scheduled independently of each other.
 *
 * REQUIRED CONSTANTS (in ShooterConstants, colocated in this package --
 * move/duplicate these out of the old TurretConstants' flywheel fields):
 *   kFlywheelKP/KI/KD/KS/KV/KA, kFlywheelToleranceRPM, kFlywheelMinSpinningRPM
 *
 * Flywheel tuning process:
 * 1. Run flywheel SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class ShooterSubsystem extends SubsystemBase {
  // Flywheel hardware - leader and follower
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;
  private final TalonFXConfiguration flywheelConfig;

  // Flywheel control request
  private final VelocityVoltage flywheelVelocityRequest;

  // Cached target (for telemetry)
  private double targetFlywheelRPM = 0.0;

  // SysId routine
  private final SysIdRoutine flywheelSysIdRoutine;

  /**
   * Creates a new ShooterSubsystem
   */
  public ShooterSubsystem() {
    // Initialize flywheel hardware
    flywheelLeader = new TalonFX(CANConstants.kFlywheelLeaderMotorID);
    flywheelFollower = new TalonFX(CANConstants.kFlywheelFollowerMotorID);
    flywheelConfig = new TalonFXConfiguration();

    // Initialize control request
    flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure motors
    configureFlywheelMotors();

    // Initialize SysId routine (leader motor only)
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

    // Set default command
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Shooter", this);

    // Output initialization progress
    Utils.logInfo("Shooter subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles the control loop onboard
  }

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

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
      .withKP(ShooterConstants.kFlywheelKP)
      .withKI(ShooterConstants.kFlywheelKI)
      .withKD(ShooterConstants.kFlywheelKD)
      .withKS(ShooterConstants.kFlywheelKS)
      .withKV(ShooterConstants.kFlywheelKV)
      .withKA(ShooterConstants.kFlywheelKA);

    // Apply configuration to both leader and follower
    flywheelLeader.getConfigurator().apply(flywheelConfig);
    flywheelFollower.getConfigurator().apply(flywheelConfig);

    // Configure follower to mirror leader
    flywheelFollower.setControl(new Follower(CANConstants.kFlywheelLeaderMotorID, MotorAlignmentValue.Opposed));

    // Optimize CAN status frames on leader
    flywheelLeader.getVelocity().setUpdateFrequency(100.0);
    flywheelLeader.getMotorVoltage().setUpdateFrequency(50.0);
    flywheelLeader.getSupplyCurrent().setUpdateFrequency(50.0);
    flywheelLeader.getTorqueCurrent().setUpdateFrequency(50.0);
    flywheelLeader.getDeviceTemp().setUpdateFrequency(4.0);
    flywheelLeader.optimizeBusUtilization();

    // Minimize follower CAN traffic
    flywheelFollower.getVelocity().setUpdateFrequency(100.0);
    flywheelFollower.getMotorVoltage().setUpdateFrequency(50.0);
    flywheelFollower.getSupplyCurrent().setUpdateFrequency(50.0);
    flywheelFollower.getTorqueCurrent().setUpdateFrequency(50.0);
    flywheelFollower.getDeviceTemp().setUpdateFrequency(4.0);
    flywheelFollower.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
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
    return Math.abs(targetFlywheelRPM - getFlywheelRPM()) < ShooterConstants.kFlywheelToleranceRPM;
  }

  /**
   * Gets whether the flywheel is spinning (above a minimum threshold)
   * @return True if spinning
   */
  private boolean isFlywheelSpinning() {
    return getFlywheelRPM() > ShooterConstants.kFlywheelMinSpinningRPM;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isFlywheelAtTargetTrigger = new Trigger(this::isFlywheelAtTarget);
  public final Trigger isFlywheelSpinningTrigger = new Trigger(this::isFlywheelSpinning);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the shooter at the start of the autonomous phase.
   */
  public void autonomousInit() {
    stopFlywheel();
    Utils.logInfo("Shooter subsystem initialized for autonomous");
  }

  /**
   * Initializes the shooter at the start of the teleop phase.
   */
  public void teleopInit() {
    stopFlywheel();
    Utils.logInfo("Shooter subsystem initialized for teleop");
  }

  /**
   * Initializes the shooter for post match (disabled) state.
   */
  public void postMatch() {
    stopFlywheel();
    Utils.logInfo("Shooter subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return flywheelSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheelSysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

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
  public Command stopCommand() {
    return runOnce(this::stopFlywheel);
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target RPM",         () -> Utils.showDouble(targetFlywheelRPM), null);
    builder.addDoubleProperty("Current RPM",        () -> Utils.showDouble(getFlywheelRPM()), null);
    builder.addDoubleProperty("RPM Error",          () -> Utils.showDouble(targetFlywheelRPM - getFlywheelRPM()), null);
    builder.addBooleanProperty("At Target",         this::isFlywheelAtTarget, null);
    builder.addBooleanProperty("Spinning",          this::isFlywheelSpinning, null);
    builder.addDoubleProperty("Leader Voltage (V)", () -> Utils.showDouble(flywheelLeader.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Current (A)", () -> Utils.showDouble(flywheelLeader.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Temp (C)",    () -> Utils.showDouble(flywheelLeader.getDeviceTemp().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Current (A)", () -> Utils.showDouble(flywheelFollower.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Temp (C)",  () -> Utils.showDouble(flywheelFollower.getDeviceTemp().getValueAsDouble()), null);
  }
}
