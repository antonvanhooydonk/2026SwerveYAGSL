// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

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

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.util.TalonFXFactory;
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
  private final DoubleSolenoid deploySolenoid;

  // Roller control request
  private final VelocityVoltage rollerVelocityRequest;

  // Cached target (for telemetry)
  private double targetRPM = 0.0;
  private boolean deployed = false;
  
  // SysId routine
  private final SysIdRoutine rollerSysIdRoutine;

  // Tuning constants from the SmartDashboard (for real-time tuning)
  private double[] lastTuning = new double[0];

  /**
   * Creates a new IntakeSubsystem
   */
  public IntakeSubsystem() {
    // Create roller motor
    rollerMotor = TalonFXFactory.createMotor(CANConstants.kIntakeRollerMotorID, getMotorConfig());
    
    // Initialize deploy/retract solenoid
    deploySolenoid = new DoubleSolenoid(
      CANConstants.kIntakeSolenoidMotorID,
      PneumaticsModuleType.REVPH,
      1,
      2
    );

    // Initialize control request
    rollerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Initialize SysId routine (leader motor only)
    rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> rollerMotor.setControl(new VoltageOut(volts.in(Volts))),
        log -> log.motor("roller")
          .voltage(Volts.of(rollerMotor.getMotorVoltage().getValueAsDouble()))
          .angularPosition(Rotations.of(rollerMotor.getPosition().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(rollerMotor.getVelocity().getValueAsDouble())),
        this
      )
    );
    
    // Ensure the intake is retracted at startup
    retract();

    // Set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Intake", this);

    // Initialize tuning constants on the SmartDashboard for real-time tuning
    initTuningDashboard();

    // Output initialization progress
    Utils.logInfo("Intake subsystem initialized");
  }

  @Override
  public void periodic() {
    getTuningFromDashboard();
  }

  /**
   * Initialize tuning constants on the SmartDashboard for real-time tuning.
   * This method is called once during subsystem initialization.
   */
  private void initTuningDashboard() {
    if (TuningConstants.kTuneIntake) {
      SmartDashboard.putNumber("Intake/P", IntakeConstants.kP);
      SmartDashboard.putNumber("Intake/I", IntakeConstants.kI);
      SmartDashboard.putNumber("Intake/D", IntakeConstants.kD);
      SmartDashboard.putNumber("Intake/kS", IntakeConstants.kS);
      SmartDashboard.putNumber("Intake/kV", IntakeConstants.kV);
      SmartDashboard.putNumber("Intake/kA", IntakeConstants.kA);
    }
  }

  /**
   * Read tuning constants from SmartDashboard and apply them to the climber motor.
   * This allows for real-time tuning of PIDF constants without redeploying code.
   */
  private void getTuningFromDashboard() {
    if (!TuningConstants.kTuneIntake) return;

    // Get the current tuning values from the SmartDashboard
    double[] current = {
      SmartDashboard.getNumber("Intake/P", IntakeConstants.kP),
      SmartDashboard.getNumber("Intake/I", IntakeConstants.kI),
      SmartDashboard.getNumber("Intake/D", IntakeConstants.kD),
      SmartDashboard.getNumber("Intake/kS", IntakeConstants.kS),
      SmartDashboard.getNumber("Intake/kV", IntakeConstants.kV),
      SmartDashboard.getNumber("Intake/kA", IntakeConstants.kA),
    };
    if (Arrays.equals(current, lastTuning)) return;

    // Cache the current tuning values to avoid unnecessary updates
    lastTuning = current;

    // Update the controller with the new constants
    TalonFXConfiguration tuneConfig = new TalonFXConfiguration();
    tuneConfig.Slot0
      .withKP(current[0])
      .withKI(current[1])
      .withKD(current[2])
      .withKS(current[3])
      .withKV(current[4])
      .withKA(current[5]);
    
    TalonFXFactory.applyConfig(rollerMotor, tuneConfig);
  }
  
  /**
   * Configure the subsystem motors with the appropriate settings
   */
  private TalonFXConfiguration getMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set basic motor configuration parameters
    config.MotorOutput
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    // Set current limits
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(30)
      .withSupplyCurrentLowerLimit(25)
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
      .withKP(IntakeConstants.kP)
      .withKI(IntakeConstants.kI)
      .withKD(IntakeConstants.kD)
      .withKS(IntakeConstants.kS)
      .withKV(IntakeConstants.kV)
      .withKA(IntakeConstants.kA);

    // Return the motor configuration
    return config;
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
    targetRPM = MathUtil.clamp(
      rpm, 
      IntakeConstants.kRollerMinRPM, 
      IntakeConstants.kRollerMaxRPM
    );

    // Set the motor control request (TalonFX velocity is in RPS)
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(targetRPM / 60.0));
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
    deployed = true;
  }

  /**
   * Retracts the intake
   */
  private void retract() {
    deploySolenoid.set(Value.kReverse);
    deployed = false;
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
  private boolean isDeployed() {
    return deployed;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isDeployedTrigger = new Trigger(this::isDeployed)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the roller at the start of the autonomous phase.
   */
  public void autonomousInit() {
    setRollerRPM(0.0);
    Utils.logInfo("Intake subsystem initialized for autonomous");
  }

  /**
   * Initializes the roller at the start of the teleop phase.
   */
  public void teleopInit() {
    setRollerRPM(0.0);
    Utils.logInfo("Intake subsystem initialized for teleop");
  }

  /**
   * Initializes the roller for post match (disabled) state.
   */
  public void postMatch() {
    setRollerRPM(0.0);
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
  public Command inCommand() {
    return run(() -> setRollerRPM(IntakeConstants.kRollerForwardRPM))
      .withName("Intake_In");
  }

  /**
   * Command to run the roller in reverse.
   * @return Command to run the roller in reverse
   */
  public Command outCommand() {
    return run(() -> setRollerRPM(IntakeConstants.kRollerReverseRPM))
      .withName("Intake_Out");
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
    super.initSendable(builder);
    builder.addBooleanProperty("Deployed",   this::isDeployed, null);
    builder.addDoubleProperty("Target RPM",  () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current RPM", () -> Utils.showDouble(getRollerRPM()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(rollerMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Temp (C)",    () -> Utils.showDouble(rollerMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
