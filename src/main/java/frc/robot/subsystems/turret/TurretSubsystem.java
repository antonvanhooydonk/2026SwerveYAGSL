// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.TuningConstants;
import frc.robot.util.Conversions;
import frc.robot.util.ShotCalculator;
import frc.robot.util.TalonFXFactory;
import frc.robot.util.Utils;

/**
 * Turret rotation subsystem using a Falcon 500 (TalonFX). Rotation only --
 * the flywheel shooter mounted on the turret is a separate ShooterSubsystem
 * so the two can be commanded and scheduled independently.
 *
 * NOTE: This subsystem assumes the turret has a LIMITED mechanical range of
 * motion (no slip rings) bounded by TurretConstants.kMinAngleDegrees and
 * kMaxAngleDegrees. setAngle() will refuse to wind past that range even
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

  // Tuning constants from the SmartDashboard (for real-time tuning)
  private double[] lastTuning = new double[0];

  // Simulation (null on a real robot)
  private DCMotorSim turretSim;
  private TalonFXSimState turretMotorSim;
  private StructPublisher<Pose3d> simPosePublisher;

  /**
   * Creates a new TurretSubsystem
   */
  public TurretSubsystem() {
    // Create the turret motor with the given configuration
    turretMotor = TalonFXFactory.createMotor(CANConstants.kTurretMotorID, getMotorConfig());

    // Initialize control request
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // NOTE: Do NOT zero the encoder here. The TalonFX keeps its position across a robot code
    // restart (crash, brownout of the roboRIO, redeploy) and only resets to 0 on a power cycle.
    // Zeroing in the constructor would silently shift the soft limits whenever the code restarts
    // with the turret away from home. Use setHomePositionCommand() (disabled-only) instead.
    turretMotor.getPosition().waitForUpdate(0.25); // make sure the logged value is not a stale 0
    Utils.logInfo("Turret startup angle (deg): " + Utils.showDouble(getAngleDegrees()));

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

    // Initialize tuning constants on the SmartDashboard for real-time tuning
    initTuningDashboard();

    // Initialize simulation if running in simulation mode
    if (RobotBase.isSimulation()) {
      initSimulation();
    }

    // Output initialization progress
    Utils.logInfo("Turret subsystem initialized");
  }

  @Override
  public void periodic() {
    getTuningFromDashboard();
  }

  @Override
  public void simulationPeriodic() {
    turretMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    turretSim.setInputVoltage(turretMotorSim.getMotorVoltage());
    turretSim.update(0.020);

    // Hard stops just past the soft limits, so momentum can't carry the turret through them
    double minRad = Units.degreesToRadians(TurretConstants.kMinAngleDegrees);
    double maxRad = Units.degreesToRadians(TurretConstants.kMaxAngleDegrees);
    double pos = turretSim.getAngularPositionRad();
    double vel = turretSim.getAngularVelocityRadPerSec();
    if ((pos < minRad && vel < 0) || (pos > maxRad && vel > 0)) {
      turretSim.setState(MathUtil.clamp(pos, minRad, maxRad), 0.0);
    }

    turretMotorSim.setRawRotorPosition(Conversions.degreesToRotations(
      turretSim.getAngularPositionRad(), 
      TurretConstants.kGearRatio
    ));
    turretMotorSim.setRotorVelocity(Conversions.degreesToRotations(
      turretSim.getAngularVelocityRadPerSec(), 
      TurretConstants.kGearRatio
    ));

    // Robot-relative turret rotation for AdvantageScope's 3D robot model
    simPosePublisher.set(new Pose3d(0, 0, 0, new Rotation3d(0, 0, turretSim.getAngularPositionRad())));
  }

  /**
   * Initialize the simulation for the turret subsystem. This sets 
   * up the physics model and visualization for simulation mode.
   */
  private void initSimulation() {
    turretMotorSim = turretMotor.getSimState();

    // Must match the InvertedValue in getMotorConfig()
    turretMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;

    DCMotor gearbox = DCMotor.getFalcon500(1);
    turretSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        gearbox,
        TurretConstants.kMOIKgMetersSq,
        TurretConstants.kGearRatio
      ),
      gearbox
    );

    // Start at home (0), matching the TalonFX's power-on position
    turretSim.setState(0.0, 0.0);
    turretMotorSim.setRawRotorPosition(0.0);

    simPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("Sim/TurretPose", Pose3d.struct)
      .publish();
  }

  /**
   * Initialize tuning constants on the SmartDashboard for real-time tuning.
   * This method is called once during subsystem initialization.
   */
  private void initTuningDashboard() {
    if (TuningConstants.kTuneTurret) {
      SmartDashboard.putNumber("Turret/P", TurretConstants.kP);
      SmartDashboard.putNumber("Turret/I", TurretConstants.kI);
      SmartDashboard.putNumber("Turret/D", TurretConstants.kD);
      SmartDashboard.putNumber("Turret/kS", TurretConstants.kS);
      SmartDashboard.putNumber("Turret/kV", TurretConstants.kV);
      SmartDashboard.putNumber("Turret/kA", TurretConstants.kA);
    }
  }

  /**
   * Read tuning constants from SmartDashboard and apply them to the climber motor.
   * This allows for real-time tuning of PIDF constants without redeploying code.
   */
  private void getTuningFromDashboard() {
    if (!TuningConstants.kTuneTurret) return;

    // Get the current tuning values from the SmartDashboard
    double[] current = {
      SmartDashboard.getNumber("Turret/P", TurretConstants.kP),
      SmartDashboard.getNumber("Turret/I", TurretConstants.kI),
      SmartDashboard.getNumber("Turret/D", TurretConstants.kD),
      SmartDashboard.getNumber("Turret/kS", TurretConstants.kS),
      SmartDashboard.getNumber("Turret/kV", TurretConstants.kV),
      SmartDashboard.getNumber("Turret/kA", TurretConstants.kA)
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
    
    TalonFXFactory.applyConfig(turretMotor, tuneConfig);
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
        TurretConstants.kGearRatio
      ))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(Conversions.degreesToRotations(
        TurretConstants.kMinAngleDegrees,
        TurretConstants.kGearRatio
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
  private void setAngle(double angleDegrees) {
    // Clamp target to valid range
    targetAngleDegrees = MathUtil.clamp(
      Utils.normalizeAngleDegrees(angleDegrees), 
      TurretConstants.kMinAngleDegrees, 
      TurretConstants.kMaxAngleDegrees
    );

    // Convert target angle to motor rotations
    double targetMotorRotations = Conversions.degreesToRotations(
      targetAngleDegrees, 
      TurretConstants.kGearRatio
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
    if (isAtAngle(TurretConstants.kMaxAngleDegrees) && volts > 0) {
      stop();
      return;
    }

    // Check if the turret is at the lower limit and trying to move down
    if (isAtAngle(TurretConstants.kMinAngleDegrees) && volts < 0) {
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
      TurretConstants.kGearRatio
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
   * Checks if the turret is at its target angle within tolerance
   * @return True if at target
   */
  private boolean isAtAngle() {
    return isAtAngle(targetAngleDegrees);
  }

  /**
   * Gets whether the turret is at the specified target angle within tolerance
   * @param targetAngleDegrees The target angle in degrees
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
   * Gets the turret's pose in field coordinates by applying a transform to the robot's pose.
   * @param robotPose The current pose of the robot in field coordinates
   * @return The pose of the turret in field coordinates
   */
  public Pose2d getPose(Pose2d robotPose) {
    return new Pose2d(
      robotPose.getTranslation().plus(TurretConstants.kRobotToTurret.rotateBy(robotPose.getRotation())),
      robotPose.getRotation()
    );
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------
  
  public final Trigger isAtTargetTrigger = new Trigger(this::isAtAngle)
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtUpperLimitTrigger = new Trigger(() -> isAtAngle(TurretConstants.kMaxAngleDegrees))
    .debounce(0.1, Debouncer.DebounceType.kRising);

  public final Trigger isAtLowerLimitTrigger = new Trigger(() -> isAtAngle(TurretConstants.kMinAngleDegrees))
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
  public Command setAngleCommand(double angleDegrees) {
    return run(() -> setAngle(angleDegrees))
      .until(this::isAtAngle)
      .withTimeout(TurretConstants.kMoveTimeoutSeconds)
      .finallyDo(this::stop)
      .withName("Turret_setAngle");
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
      setAngle(turretAngle);
    })
    .withName("Turret_AimAtFieldAngle");
  }

  /**
   * Command to continuously aim the turret at a target pose on the field.
   * @param turretPoseSupplier Supplier for the turret's current field pose
   * @param targetPoseSupplier Supplier for the field-relative target pose to face
   * @return Command to continuously aim at the target pose
   */
  public Command aimAtPoseCommand(
    Supplier<Pose2d> turretPoseSupplier,
    Supplier<Pose2d> targetPoseSupplier
  ) {
    return aimAtPoseCommand(turretPoseSupplier, targetPoseSupplier, null);
  }

  /**
   * Command to continuously aim the turret at a target pose on the field 
   * and compensate for the robot's movement while doing so.
   * @param turretPoseSupplier Supplier for the turret's current field pose
   * @param targetPoseSupplier Supplier for the field-relative target pose to face
   * @param fieldSpeedsSupplier Supplier for the robot's current field-relative chassis speeds
   * @return Command to continuously aim at the target pose
   */
  public Command aimAtPoseCommand(
    Supplier<Pose2d> turretPoseSupplier,
    Supplier<Pose2d> targetPoseSupplier,
    Supplier<ChassisSpeeds> fieldSpeedsSupplier
  ) {
    return run(() -> {
      Pose2d turretPose = turretPoseSupplier == null ? null : turretPoseSupplier.get();
      Pose2d targetPose = targetPoseSupplier == null ? null : targetPoseSupplier.get();
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier == null ? null : fieldSpeedsSupplier.get();

      // If any input pose is null, we can't calculate the angle, so stop the turret and exit early
      if (turretPose == null || targetPose == null) {
        stop();
        return;
      }

      // Same solution the flywheel uses, so aim and RPM always agree
      ShotCalculator.Solution shot = ShotCalculator.solve(turretPose, targetPose.getTranslation(), fieldSpeeds);

      // Angle to the VIRTUAL target in field coordinates, converted to robot-relative
      double dx = shot.virtualTarget().getX() - turretPose.getX();
      double dy = shot.virtualTarget().getY() - turretPose.getY();
      double fieldAngleDegrees = Units.radiansToDegrees(Math.atan2(dy, dx));
      double turretAngle = fieldAngleDegrees - turretPose.getRotation().getDegrees();

      // Command the turret to the calculated angle
      setAngle(turretAngle);
    })
    .withName("Turret_AimAtPose");
  }

  /**
   * Command to zero the turret encoder at its current position. Only run this while the
   * turret is physically at its home position (pit use, disabled only).
   * @return Command that resets the encoder
   */
  public Command setHomePositionCommand() {
    return runOnce(this::resetEncoder)
      .ignoringDisable(true)
      .withName("Turret_SetHomePosition");
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
    super.initSendable(builder);
    builder.addDoubleProperty("Target Angle (deg)",  () -> Utils.showDouble(targetAngleDegrees), null);
    builder.addDoubleProperty("Current Angle (deg)", () -> Utils.showDouble(getAngleDegrees()), null);
    builder.addDoubleProperty("Current (A)",         () -> Utils.showDouble(turretMotor.getSupplyCurrent().getValueAsDouble()), null);
    builder.addDoubleProperty("Temp (C)",            () -> Utils.showDouble(turretMotor.getDeviceTemp().getValueAsDouble()), null);
  }
}
