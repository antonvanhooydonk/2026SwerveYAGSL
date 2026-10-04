// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANConstants;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.util.TalonFXFactory;
import frc.robot.util.TalonFXFactory.MotorPair;
import frc.robot.util.Utils;

/**
 * Flywheel subsystem using dual Kraken X60 (TalonFX) motors in a
 * follower configuration. Physically mounted on the turret, but kept as a
 * separate subsystem from TurretSubsystem so aiming and spin-up can be
 * commanded and scheduled independently of each other.
 *
 * Flywheel tuning process:
 * 1. Run flywheel SysId to characterize kS, kV, kA
 * 2. Start with feedforward only (kP = 0)
 * 3. Add minimal kP if steady-state error remains (usually 0.05 - 0.2)
 * 4. Avoid kI and kD unless absolutely necessary
 */
public class FlywheelSubsystem extends SubsystemBase {
  // Flywheel hardware - leader and follower
  private final TalonFX flywheelLeader;
  private final TalonFX flywheelFollower;

  // Flywheel control request
  private final VelocityVoltage flywheelVelocityRequest;

  // Cached target (for telemetry)
  private double targetRPM = 0.0;

  // A TreeMap where Key = Distance (meters) and Value = flywheel RPM
  private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();

  // SysId routine
  private final SysIdRoutine flywheelSysIdRoutine;

  /**
   * Creates a new FlywheelSubsystem
   */
  public FlywheelSubsystem() {
    // Create leader and follower motors
    MotorPair flywheelMotors = TalonFXFactory.createMotorPair(
      CANConstants.kFlywheelLeaderMotorID, 
      CANConstants.kFlywheelFollowerMotorID, 
      getMotorConfig(), 
      true
    );
    flywheelLeader = flywheelMotors.leader();
    flywheelFollower = flywheelMotors.follower();

    // Initialize control request
    flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Initialize SysId routine (leader motor only)
    flywheelSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, null),
      new SysIdRoutine.Mechanism(
        volts -> flywheelLeader.setControl(new VoltageOut(volts.in(Volts))),
        log -> log.motor("flywheel")
          .voltage(Volts.of(flywheelLeader.getMotorVoltage().getValueAsDouble()))
          .angularPosition(Rotations.of(flywheelLeader.getPosition().getValueAsDouble()))
          .angularVelocity(RotationsPerSecond.of(flywheelLeader.getVelocity().getValueAsDouble())),
        this
      )
    );

    // Initialize RPM table for distance-based shooting
    initializeRPMTable();
    
    // set the default command for this subsystem
    setDefaultCommand(stopCommand());

    // Add data to dashboard
    SmartDashboard.putData("Flywheel", this);

    // Output initialization progress
    Utils.logInfo("Flywheel subsystem initialized");
  }

  /**
   * Initializes the RPM table with distanceMeters-RPM pairs for interpolation
   */
  private void initializeRPMTable() {
    rpmTable.put(0.0, 1000.0);
    rpmTable.put(1.0, 2000.0); 
    rpmTable.put(2.0, 2500.0);
    rpmTable.put(3.0, 3000.0); 
    rpmTable.put(4.0, 3500.0); 
    rpmTable.put(5.0, 4000.0); 
    rpmTable.put(6.0, 4500.0); 
    rpmTable.put(7.0, 5000.0); 
    rpmTable.put(8.0, 5500.0); 
    rpmTable.put(9.0, 5500.0); 
    rpmTable.put(10.0, 6000.0); 
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
      .withNeutralMode(NeutralModeValue.Coast)
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withDutyCycleNeutralDeadband(0.001);

    // Set current limits
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(40)
      .withSupplyCurrentLowerLimit(40)
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
      .withKP(FeederConstants.kP)
      .withKI(FeederConstants.kI)
      .withKD(FeederConstants.kD)
      .withKS(FeederConstants.kS)
      .withKV(FeederConstants.kV)
      .withKA(FeederConstants.kA);

    // Return the motor configuration
    return config;
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the flywheel to a target velocity in RPM
   * @param rpm Target velocity in RPM
   */
  private void setRPM(double rpm) {
    // Clamp target to valid range
    targetRPM = MathUtil.clamp(
      rpm, 
      FlywheelConstants.kFlywheelMinRPM, 
      FlywheelConstants.kFlywheelMaxRPM
    );

    // Set the flywheel velocity (TalonFX velocity is in RPS, convert RPM to RPS)
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(targetRPM / 60.0));
  }

  /**
   * Gets the current flywheel velocity in RPM
   * @return Current velocity in RPM
   */
  private double getRPM() {
    // TalonFX velocity is in RPS, convert to RPM
    return flywheelLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Stops the flywheel
   */
  private void stop() {
    targetRPM = 0.0;
    flywheelLeader.stopMotor();
  }

  /**
   * Check if flywheel is at the current target RPM within tolerance
   * @return true if within tolerance of the target RPM  
   */
  private boolean isAtTargetRPM() {
    return Math.abs(targetRPM) > 0.0 && MathUtil.isNear(
      targetRPM,
      getRPM(),
      FlywheelConstants.kFlywheelToleranceRPM
    );
  }

  /**
   * Gets whether the flywheel is spinning (above a minimum threshold)
   * @return True if spinning
   */
  private boolean isSpinning() {
    return getRPM() > FlywheelConstants.kFlywheelMinSpinningRPM;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isFlywheelAtTargetTrigger = new Trigger(this::isAtTargetRPM)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isFlywheelSpinningTrigger = new Trigger(this::isSpinning)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the flywheel at the start of the autonomous phase.
   */
  public void autonomousInit() {
    stop();
    Utils.logInfo("Flywheel subsystem initialized for autonomous");
  }

  /**
   * Initializes the flywheel at the start of the teleop phase.
   */
  public void teleopInit() {
    stop();
    Utils.logInfo("Flywheel subsystem initialized for teleop");
  }

  /**
   * Initializes the flywheel for post match (disabled) state.
   */
  public void postMatch() {
    stop();
    Utils.logInfo("Flywheel subsystem initialized for post match");
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
   * Command to spin the flywheel to a target velocity without waiting.
   * Useful when pre-spinning during aiming.
   * @param rpm Target velocity in RPM
   * @return Command to set flywheel velocity
   */
  public Command setRPMCommand(double rpm) {
    return run(() -> setRPM(rpm))
      .withName("Flywheel_setRPM");
  }

  /**
   * Command to stop the flywheel.
   * @return Command to stop the flywheel
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Flywheel_Stop");
  }

  /**
   * Command to shoot at the current target pose by calculating the  
   * required flywheel speed based on the distance to the target.
   * @param robotPoseSupplier The supplier for the current pose of the robot
   * @param targetPoseSupplier The supplier for the pose of the target (usually an alliance hub)
   * @return Command to shoot at the target
   */
  public Command shootAtPoseCommand(
    Supplier<Pose2d> robotPoseSupplier, 
    Supplier<Pose2d> targetPoseSupplier
  ) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier == null ? null : robotPoseSupplier.get();
      Pose2d targetPose = targetPoseSupplier == null ? null : targetPoseSupplier.get();

      // Default distance if either pose is null so we don't prevent shooting
      double distanceToTarget = FlywheelConstants.kFlywheelDefaultDistanceToTarget; 

      // Calculate distance to target
      if (robotPose != null && targetPose != null) {
        distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      }

      // Interpolated flywheel speed based on distance to target
      double requiredRPM = rpmTable.get(distanceToTarget);

      // Set flywheel speed
      setRPM(requiredRPM);
    })
    .withName("Flywheel_ShootAtPose");
  }

  // ----------------------------------------------------------------------------------------
  // Sendable / Dashboard
  // ----------------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target RPM",         () -> Utils.showDouble(targetRPM), null);
    builder.addDoubleProperty("Current RPM",        () -> Utils.showDouble(getRPM()), null);
    builder.addDoubleProperty("RPM Error",          () -> Utils.showDouble(targetRPM - getRPM()), null);
    builder.addBooleanProperty("At Target",         this::isAtTargetRPM, null);
    builder.addBooleanProperty("Spinning",          this::isSpinning, null);
    builder.addDoubleProperty("Leader Voltage (V)", () -> Utils.showDouble(flywheelLeader.getMotorVoltage().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Current (A)", () -> Utils.showDouble(flywheelLeader.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Leader Temp (C)",    () -> Utils.showDouble(flywheelLeader.getDeviceTemp().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Current (A)", () -> Utils.showDouble(flywheelFollower.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Follower Temp (C)",  () -> Utils.showDouble(flywheelFollower.getDeviceTemp().getValueAsDouble()), null);
  }
}
