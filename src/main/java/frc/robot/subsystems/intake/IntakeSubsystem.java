// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.util.Utils;

/**
 * Intake subsystem using a single Kraken X60 (TalonFX) motor. 
 *
 * Roller tuning process:
 * 1. Run intake SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class IntakeSubsystem extends SubsystemBase {
  // Intake hardware
  private final TalonFX rollerMotor;
  private final TalonFXConfiguration rollerConfig;
  private final DoubleSolenoid deploySolenoid;

  // Roller control request
  private final VelocityVoltage rollerVelocityRequest;

  // Cached target (for telemetry)
  private double targetRPM = 0.0;
  private boolean isDeployed = false;
  
  // SysId routine
  private final SysIdRoutine rollerSysIdRoutine;

  /**
   * Creates a new IntakeSubsystem
   */
  public IntakeSubsystem() {
    // Initialize roller hardware
    rollerMotor = new TalonFX(1);
    rollerConfig = new TalonFXConfiguration();
    
    // Initialize deploy/retract solenoid
    deploySolenoid = new DoubleSolenoid(
      0,
      PneumaticsModuleType.REVPH,
      1,
      2
    );

    // Initialize control request
    rollerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure motors
    configureMotor();

    // Initialize SysId routine (leader motor only)
    rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("roller-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> rollerMotor.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Intake", this);

    // Output initialization progress
    Utils.logInfo("Intake subsystem initialized");
  }

  @Override
  public void periodic() {
    // Nothing needed - TalonFX handles the control loop onboard
  }

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

  /**
   * Configures the roller leader motor
   */
  private void configureMotor() {
    rollerConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast) // Coast so roller spins down naturally
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    rollerConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60)
      .withSupplyCurrentLowerLimit(40)
      .withSupplyCurrentLowerTime(0.5)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);

    rollerConfig.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12)
      .withSupplyVoltageTimeConstant(0.02);

    // Velocity PID (slot 0) - velocity in RPS
    rollerConfig.Slot0
      .withKP(IntakeConstants.kRollerKP)
      .withKI(IntakeConstants.kRollerKI)
      .withKD(IntakeConstants.kRollerKD)
      .withKS(IntakeConstants.kRollerKS)
      .withKV(IntakeConstants.kRollerKV)
      .withKA(IntakeConstants.kRollerKA);

    // Apply configuration to roller motor
    rollerMotor.getConfigurator().apply(rollerConfig);
    
    // Optimize CAN status frames on roller motor
    rollerMotor.getVelocity().setUpdateFrequency(100.0);
    rollerMotor.getMotorVoltage().setUpdateFrequency(50.0);
    rollerMotor.getSupplyCurrent().setUpdateFrequency(50.0);
    rollerMotor.getTorqueCurrent().setUpdateFrequency(50.0);
    rollerMotor.getDeviceTemp().setUpdateFrequency(4.0);
    rollerMotor.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the roller to a target velocity in RPM
   * @param rpm Target velocity in RPM
   */
  private void setRollerRPM(double rpm) {
    // Clamp target to valid range
    double clampedRPM = MathUtil.clamp(
      rpm, 
      IntakeConstants.kRollerMinRPM, 
      IntakeConstants.kRollerMaxRPM
    );

    // Update cached target for telemetry
    targetRPM = clampedRPM;

    // Set the motor control request (TalonFX velocity is in RPS)
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(clampedRPM / 60.0));
  }

  /**
   * Gets the current flywheel velocity in RPM
   * @return Current velocity in RPM
   */
  private double getRollerRPM() {
    // TalonFX velocity is in RPS, convert to RPM
    return rollerMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Deploys the intake
   */
  private void deploy() {
    deploySolenoid.set(Value.kForward);
    isDeployed = true;
  }

  /**
   * Retracts the intake
   */
  private void retract() {
    deploySolenoid.set(Value.kReverse);
    isDeployed = false;
  }

  /**
   * Stops the roller
   */
  private void stopRoller() {
    rollerMotor.stopMotor();
  }

  /**
   * Returns true if the intake is deployed
   * @return true if the intake is deployed
   */
  private boolean isIntakeDeployed() {
    return isDeployed;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isDeployedTrigger = new Trigger(this::isIntakeDeployed)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the roller at the start of the autonomous phase.
   */
  public void autonomousInit() {
    targetRPM = 0.0;
    Utils.logInfo("Intake subsystem initialized for autonomous");
  }

  /**
   * Initializes the roller at the start of the teleop phase.
   */
  public void teleopInit() {
    targetRPM = 0.0;
    Utils.logInfo("Intake subsystem initialized for teleop");
  }

  /**
   * Initializes the roller for post match (disabled) state.
   */
  public void postMatch() {
    targetRPM = 0.0;
    Utils.logInfo("Intake subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return rollerSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return rollerSysIdRoutine.dynamic(direction);
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------

  /**
   * Command to deploy the intake.
   * @return Command to deploy the intake
   */
  public Command deployCommand() {
    return runOnce(this::deploy)
      .withName("Intake_Deploy");
  }

  /**
   * Command to retract the intake.
   * @return Command to retract the intake
   */
  public Command retractCommand() {
    return runOnce(this::retract)
      .withName("Intake_Retract");
  }

  /**
   * Command to run the intake in the forward direction.
   * @return Command to run the intake in the forward direction
   */
  public Command pickupCommand() {
    return run(() -> setRollerRPM(IntakeConstants.kRollerForwardRPM))
      .withName("Intake_Pickup");
  }

  /**
   * Command to run the roller in reverse.
   * @return Command to run the roller in reverse
   */
  public Command reverseCommand() {
    return run(() -> setRollerRPM(IntakeConstants.kRollerReverseRPM))
      .withName("Intake_Reverse");
  }

  /**
   * Command to stop the roller.
   * @return Command to stop the roller
   */
  public Command stopCommand() {
    return run(this::stopRoller)
      .withName("Intake_Stop");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Deployed",   this::isIntakeDeployed, null);
    builder.addDoubleProperty("Target RPM",  () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current RPM", () -> Utils.showDouble(getRollerRPM()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(rollerMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Temp (C)",    () -> Utils.showDouble(rollerMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
