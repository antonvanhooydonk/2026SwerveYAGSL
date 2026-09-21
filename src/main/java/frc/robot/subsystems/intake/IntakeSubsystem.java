// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

/**
 * Intake subsystem using dual Kraken X60 (TalonFX) motors in a
 * follower configuration. 
 *
 * Roller tuning process:
 * 1. Run intake SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class IntakeSubsystem extends SubsystemBase {
  // Roller hardware - leader and follower
  private final TalonFX rollerLeader;
  private final TalonFX rollerFollower;
  private final TalonFXConfiguration rollerConfig;

  // Flywheel control request
  private final VelocityVoltage rollerVelocityRequest;
  
  // SysId routine
  private final SysIdRoutine rollerSysIdRoutine;

  /**
   * Creates a new IntakeSubsystem
   */
  public IntakeSubsystem() {
    // Initialize roller hardware
    rollerLeader = new TalonFX(1);
    rollerFollower = new TalonFX(2);
    rollerConfig = new TalonFXConfiguration();

    // Initialize control request
    rollerVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure motors
    configureRollerMotors();

    // Initialize SysId routine (leader motor only)
    rollerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        null,
        null,
        state -> SignalLogger.writeString("roller-sysid-state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        volts -> rollerLeader.setControl(new VoltageOut(volts.in(Volts))),
        null,
        this
      )
    );

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
   * Configures the roller leader and follower motors
   */
  private void configureRollerMotors() {
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
      .withKP(IntakeConstants.kFlywheelKP)
      .withKI(IntakeConstants.kFlywheelKI)
      .withKD(IntakeConstants.kFlywheelKD)
      .withKS(IntakeConstants.kFlywheelKS)
      .withKV(IntakeConstants.kFlywheelKV)
      .withKA(IntakeConstants.kFlywheelKA);

    // Apply configuration to both leader and follower
    rollerLeader.getConfigurator().apply(rollerConfig);
    rollerFollower.getConfigurator().apply(rollerConfig);

    // Configure follower to mirror leader
    rollerFollower.setControl(new Follower(1, MotorAlignmentValue.Opposed));
    
    // Optimize CAN status frames on leader
    rollerLeader.getVelocity().setUpdateFrequency(100.0);
    rollerLeader.getMotorVoltage().setUpdateFrequency(50.0);
    rollerLeader.getSupplyCurrent().setUpdateFrequency(50.0);
    rollerLeader.getTorqueCurrent().setUpdateFrequency(50.0);
    rollerLeader.getDeviceTemp().setUpdateFrequency(4.0);
    rollerLeader.optimizeBusUtilization();

    // Minimize follower CAN traffic
    rollerFollower.getVelocity().setUpdateFrequency(100.0);
    rollerFollower.getMotorVoltage().setUpdateFrequency(50.0);
    rollerFollower.getSupplyCurrent().setUpdateFrequency(50.0);
    rollerFollower.getTorqueCurrent().setUpdateFrequency(50.0);
    rollerFollower.getDeviceTemp().setUpdateFrequency(4.0);
    rollerFollower.optimizeBusUtilization();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Gets the current flywheel velocity in RPM
   * @return Current velocity in RPM
   */
  private double getRollerRPM() {
    // TalonFX velocity is in RPS, convert to RPM
    return rollerLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Sets the roller to a target velocity in RPM
   * @param rpm Target velocity in RPM
   */
  private void setRollerRPM(double rpm) {
    double rps = rpm / 60.0;
    rollerLeader.setControl(rollerVelocityRequest.withVelocity(rps));
  }

  /**
   * Forwards the roller at a fixed speed defined in IntakeConstants
   */
  private void forwardRoller() {
    setRollerRPM(IntakeConstants.kRollerForwardRPM);
  }

  /**
   * Reverses the roller at a fixed speed defined in IntakeConstants
   */
  private void reverseRoller() {
    setRollerRPM(IntakeConstants.kRollerReverseRPM);
  }

  /**
   * Stops the roller
   */
  private void stopRoller() {
    rollerLeader.stopMotor();
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  // public final Trigger isFlywheelAtTargetTrigger = new Trigger(this::isFlywheelAtTarget);


  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the roller at the start of the autonomous phase.
   */
  public void autonomousInit() {
    stopRoller();
    Utils.logInfo("Intake subsystem initialized for autonomous");
  }

  /**
   * Initializes the roller at the start of the teleop phase.
   */
  public void teleopInit() {
    stopRoller();
    Utils.logInfo("Intake subsystem initialized for teleop");
  }

  /**
   * Initializes the roller for post match (disabled) state.
   */
  public void postMatch() {
    stopRoller();
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
   * Command to stop the flywheel.
   * @return Command to stop the flywheel
   */
  public Command deployCommand() {
    return run(this::stopRoller);
  }

  /**
   * Command to stop the flywheel.
   * @return Command to stop the flywheel
   */
  public Command retractCommand() {
    return run(this::stopRoller);
  }

  /**
   * Command to stop the flywheel.
   * @return Command to stop the flywheel
   */
  public Command intakeCommand() {
    return run(this::forwardRoller);
  }

  /**
   * Command to run the roller in reverse.
   * @return Command to run the roller in reverse
   */
  public Command ejectCommand() {
    return run(this::reverseRoller);
  }

  /**
   * Command to stop the roller.
   * @return Command to stop the roller
   */
  public Command stopCommand() {
    return run(this::stopRoller);
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {

  }
}
