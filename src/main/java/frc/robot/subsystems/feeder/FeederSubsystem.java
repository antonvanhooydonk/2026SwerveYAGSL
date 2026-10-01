// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.util.Utils;

/**
 * Feeder subsystem using a Kraken X60 (TalonFX) motor. 
 *
 * Feeder tuning process:
 * 1. Run feeder SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class FeederSubsystem extends SubsystemBase {
  // Feeder hardware
  private final TalonFX feederMotor;
  private final TalonFXConfiguration feederConfig;

  // Feeder control request
  private final VelocityVoltage feederVelocityRequest;

  // Cached target (for telemetry)
  private double targetRPM = 0.0;

  // SysId routine
  private final SysIdRoutine feederSysIdRoutine;

  /**
   * Creates a new FeederSubsystem
   */
  public FeederSubsystem() {
    // Initialize feeder hardware
    feederMotor = new TalonFX(CANConstants.kFeederMotorID);
    feederConfig = new TalonFXConfiguration();

    // Initialize control request
    feederVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure motors
    configureMotors();

    // Initialize SysId routine (leader motor only)
    feederSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> feederMotor.setControl(new VoltageOut(volts.in(Volts))),
        log -> log.motor("feeder")
          .voltage(Volts.of(feederMotor.getMotorVoltage().getValueAsDouble()))
          .angularPosition(Rotations.of(feederMotor.getPosition().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(feederMotor.getVelocity().getValueAsDouble())),
        this
      )
    );

    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Feeder", this);

    // Output initialization progress
    Utils.logInfo("Feeder subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles the control loop onboard
  }

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

  /**
   * Configures the feeder motor
   */
  private void configureMotors() {
    feederConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake) // Brake so feeder stops quickly
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    feederConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)
      .withSupplyCurrentLowerLimit(40)
      .withSupplyCurrentLowerTime(1.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(60);

    feederConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Velocity PID (slot 0) - velocity in RPS
    feederConfig.Slot0
      .withKP(FeederConstants.kFeederKP)
      .withKI(FeederConstants.kFeederKI)
      .withKD(FeederConstants.kFeederKD)
      .withKS(FeederConstants.kFeederKS)
      .withKV(FeederConstants.kFeederKV)
      .withKA(FeederConstants.kFeederKA);

    // Apply configuration to feeder motor
    feederMotor.getConfigurator().apply(feederConfig);

    // Optimize CAN status frames on feeder motor
    feederMotor.getVelocity().setUpdateFrequency(100.0);
    feederMotor.getPosition().setUpdateFrequency(100.0);
    feederMotor.getMotorVoltage().setUpdateFrequency(50.0);
    feederMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    feederMotor.getTorqueCurrent().setUpdateFrequency(50.0);
    feederMotor.getDeviceTemp().setUpdateFrequency(4.0);
    feederMotor.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the feeder to a target velocity in RPM
   * @param rpm Target velocity in RPM
   */
  private void setRPM(double rpm) {
    // Clamp target to valid range
    targetRPM = MathUtil.clamp(
      rpm, 
      FeederConstants.kFeederMinRPM, 
      FeederConstants.kFeederMaxRPM
    );

    // Set the feeder velocity (TalonFX velocity is in RPS, convert RPM to RPS)
    feederMotor.setControl(feederVelocityRequest.withVelocity(targetRPM / 60.0));
  }

  /**
   * Gets the current feeder velocity in RPM
   * @return Current velocity in RPM
   */
  private double getRPM() {
    // TalonFX velocity is in RPS, convert to RPM
    return feederMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Stops the feeder
   */
  private void stop() {
    targetRPM = 0.0;
    feederMotor.stopMotor();
  }

  /**
   * Check if feeder is at the current target RPM within tolerance
   * @return true if within tolerance of the target RPM  
   */
  private boolean isAtTargetRPM() {
    return MathUtil.isNear(
      targetRPM,
      getRPM(),
      FeederConstants.kFeederToleranceRPM
    );
  }

  /**
   * Gets whether the feeder is spinning (above a minimum threshold)
   * @return True if spinning
   */
  private boolean isSpinning() {
    return getRPM() > FeederConstants.kFeederMinSpinningRPM;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isFeederAtTargetTrigger = new Trigger(this::isAtTargetRPM)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isFeederSpinningTrigger = new Trigger(this::isSpinning)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the shooter at the start of the autonomous phase.
   */
  public void autonomousInit() {
    stop();
    Utils.logInfo("Feeder subsystem initialized for autonomous");
  }

  /**
   * Initializes the feeder at the start of the teleop phase.
   */
  public void teleopInit() {
    stop();
    Utils.logInfo("Feeder subsystem initialized for teleop");
  }

  /**
   * Initializes the feeder for post match (disabled) state.
   */
  public void postMatch() {
    stop();
    Utils.logInfo("Feeder subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return feederSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return feederSysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

  /**
   * Command to spin the feeder to a target velocity without waiting.
   * Useful when pre-spinning during aiming.
   * @param rpm Target velocity in RPM
   * @return Command to set feeder velocity
   */
  public Command setRPMCommand(double rpm) {
    return run(() -> setRPM(rpm))
      .withName("Feeder_setRPM");
  }

  /**
   * Command to feed fuel in the feeder.
   * @return Command to set feeder velocity
   */
  public Command feedCommand() {
    return run(() -> setRPM(FeederConstants.kFeedRPM))
      .withName("Feeder_Feed");
  }

  /**
   * Command to reverse the feeder.
   * @return Command to set feeder velocity
   */
  public Command reverseCommand() {
    return run(() -> setRPM(FeederConstants.kReverseRPM))
      .withName("Feeder_Reverse");
  }

  /**
   * Command to stop the feeder.
   * @return Command to stop the feeder
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Feeder_Stop");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target RPM",   () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current RPM",  () -> Utils.showDouble(getRPM()), null);
    builder.addDoubleProperty("RPM Error",    () -> Utils.showDouble(targetRPM - getRPM()), null);
    builder.addBooleanProperty("At Target",   this::isAtTargetRPM, null);
    builder.addBooleanProperty("Spinning",    this::isSpinning, null);
    builder.addDoubleProperty("Voltage (V)",  () -> Utils.showDouble(feederMotor.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Current (A)",  () -> Utils.showDouble(feederMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Temp (C)",     () -> Utils.showDouble(feederMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
