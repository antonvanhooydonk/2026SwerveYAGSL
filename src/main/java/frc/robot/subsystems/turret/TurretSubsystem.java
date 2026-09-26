// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
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
  private final TalonFXConfiguration turretConfig;

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
    // Initialize turret hardware
    turretMotor = new TalonFX(CANConstants.kTurretMotorID);
    turretConfig = new TalonFXConfiguration();

    // Initialize control request
    motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Configure motor
    configureMotor();

    // Zero turret encoder at startup - turret must be at home position
    resetEncoder();

    // Initialize SysId routine
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

  // ----------------------------------------------------------------------------------------
  // Private configuration methods
  // ----------------------------------------------------------------------------------------

  /**
   * Configures the turret rotation motor
   */
  private void configureMotor() {
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

    // Do NOT use withSensorToMechanismRatio, instead use Conversions methods
    // in appropriate places within this subsystem code.

    // Software limits are the ONLY protection this turret has (no slip
    // rings, no physical limit switches). These are in raw (unwrapped)
    // mechanism degrees, which can exceed +/-180 as the turret accumulates
    // position across multiple tracking commands.
    turretConfig.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(TurretConstants.kMaxAngleDegrees)
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(TurretConstants.kMinAngleDegrees);

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

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Sets the turret to a target angle using MotionMagic, taking the shortest
   * path UNLESS that path would exceed the turret's mechanical range, in
   * which case the long way around is used instead, or the target is
   * clamped to the nearest reachable limit if neither path is safe.
   * @param angleDegrees Target angle in degrees
   */
  private void setTurretAngle(double angleDegrees) {
    double normalizedAngle = normalizeAngleDegrees(angleDegrees);

    // Convert current motor rotations to tracking angle space
    double currentRawPositionDegrees = Conversions.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble(), TurretConstants.kTurretGearRatio);
    double currentAngle = normalizeAngleDegrees(currentRawPositionDegrees);

    // Calculate the shortest path to the target angle
    double shortestDelta = normalizeAngleDegrees(normalizedAngle - currentAngle);
    double shortestPathTargetDegrees = currentRawPositionDegrees + shortestDelta;
    double targetPositionDegrees = shortestPathTargetDegrees;

    // Check if the shortest path target is out of range
    boolean shortestPathOutOfRange =
        shortestPathTargetDegrees > TurretConstants.kMaxAngleDegrees
        || shortestPathTargetDegrees < TurretConstants.kMinAngleDegrees;

    // Check if the shortest path is out of range
    if (shortestPathOutOfRange) {
      // Calculate the long path to the target angle
      double longDelta = shortestDelta > 0 ? shortestDelta - 360.0 : shortestDelta + 360.0;
      double longPathTargetDegrees = currentRawPositionDegrees + longDelta;

      // Check if the long path target is in range
      boolean longPathInRange =
          longPathTargetDegrees <= TurretConstants.kMaxAngleDegrees
          && longPathTargetDegrees >= TurretConstants.kMinAngleDegrees;

      // If the long path is in range
      if (longPathInRange) {
        // Use the long path target
        targetPositionDegrees = longPathTargetDegrees;
      } 
      else {
        // Otherwise clamp to the nearest limit
        targetPositionDegrees = MathUtil.clamp(
          shortestPathTargetDegrees,
          TurretConstants.kMinAngleDegrees,
          TurretConstants.kMaxAngleDegrees
        );
      }
    }

    // Update the cached target for telemetry
    targetAngleDegrees = normalizedAngle;

    // Convert calculation space back to native motor rotations before updating target
    double targetMotorRotations = Conversions.degreesToRotations(targetPositionDegrees, TurretConstants.kTurretGearRatio);
    
    // Command the turret motor to the target position using MotionMagic
    turretMotor.setControl(motionMagicRequest.withPosition(targetMotorRotations));
  }

  /**
   * Gets the current turret angle in degrees, normalized to (-180, 180]
   * @return Current angle in degrees
   */
  private double getAngleDegrees() {
    return normalizeAngleDegrees(Conversions.rotationsToDegrees(
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
   * Sets the turret motor to brake or coast mode
   * @param brake True for brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    turretMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /**
   * Normalizes an angle to (-180, 180]
   * @param angleDegrees Angle in degrees
   * @return Normalized angle in degrees
   */
  private double normalizeAngleDegrees(double angleDegrees) {
    return MathUtil.inputModulus(angleDegrees, -180.0, 180.0);
  }

  /**
   * Gets whether the turret is at its target angle within tolerance
   * @return True if at target
   */
  private boolean isAtAngle(double targetAngleDegrees) {
    return MathUtil.isNear(
      normalizeAngleDegrees(targetAngleDegrees),
      getAngleDegrees(),
      TurretConstants.kTurretAngleToleranceDegrees
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
    setMotorBrake(true);
    setTurretAngle(0);
    Utils.logInfo("Turret subsystem initialized for autonomous");
  }

  /**
   * Initializes the turret at the start of the teleop phase.
   */
  public void teleopInit() {
    setMotorBrake(true);
    Utils.logInfo("Turret subsystem initialized for teleop");
  }

  /**
   * Initializes the turret for post match (disabled) state.
   */
  public void postMatch() {
    setMotorBrake(false);
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
      double turretAngle = normalizeAngleDegrees(
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
      double turretAngle = normalizeAngleDegrees(fieldAngleDegrees - robotPose.getRotation().getDegrees());
      
      // Command the turret to the calculated angle
      setTurretAngle(turretAngle);
    })
    .withName("Turret_AimAtPose");
  }

  /**
   * Command to home the turret to 0 degrees.
   * @return Command to home the turret
   */
  public Command homeCommand() {
    return setTurretAngleCommand(0.0)
      .withName("Turret_Home");
  }

  /**
   * Command to stop the turret.
   */
  public Command stopCommand() {
    return run(this::stop)
      .withName("Turret_Stop");
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
