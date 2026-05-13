// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.Constants.FieldConstants;
import frc.robot.util.Utils;

import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve drive subsystem using four SwerveModules.
 * 
 * Integrated functionality:
 * 1. PathPlanner 
 * 2. PhotonVision integration via VisionSubsystem
 * 3. Slew rate limiting for smoother joystick control
 */
public class SwerveSubsystem extends SubsystemBase {
  // YAGSL swerve drive
  private final SwerveDrive swerveDrive;

  // Gyro for field-relative driving
  private final AHRS gyro;

  // Setpoint generator
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint driveSetpoint;

  // Current robot state
  private boolean fieldRelative = true;
  private boolean slowMode = false;
  private long acceptedVisionCount = 0;
  private long rejectedVisionCount = 0;

  /**
   * Creates a new SwerveSubsystem
   * @param visionSubsystem The vision subsystem for pose estimation
   */
  public SwerveSubsystem(File directory) {
    // Determine starting pose based on alliance color.
    Pose2d startingPose = Utils.isRedAlliance() 
      ? new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)), Rotation2d.fromDegrees(180)) 
      : new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0));

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    // Initialize YAGSL SwerveDrive
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.kMaxSpeedMetersPerSecond, startingPose);
    } catch (Exception e) {
      throw new RuntimeException("FAILED TO INITIALIZE SWERVE DRIVE!!!", e);
    }

    // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setHeadingCorrection(false);
    
    // Disables cosine compensation as recommended when using swerveDrive.drive(robotVelocity, state, feedforwards)
    swerveDrive.setCosineCompensator(false);

    // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    
    // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    
    // Stop the odometry thread if we are using vision that way we can synchronize updates better.
    swerveDrive.stopOdometryThread();

    // Get the gyro from the swerve drive
    gyro = (AHRS)swerveDrive.getGyro().getIMU();

    // Initialize the swerve setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(
      SwerveConstants.kRobotConfig, 
      SwerveConstants.kMaxAngularSpeedRadsPerSecond
    );
    
    // Initialize the drive setpoint
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      (pose) -> resetOdometry(pose), // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
      ),
      SwerveConstants.kRobotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD THEM HERE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Add data to dashboard
    SmartDashboard.putData("Drive", this);

    // Output initialization progress
    Utils.logInfo("Drive subsystem initialized");
  }

  @Override
  public void periodic() {
    // Always update odometry manually in order to allow vision updates to be integrated correctly
    swerveDrive.updateOdometry();
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Drive the robot using the given robot-relative chassis speeds.
   * @param speeds The desired robot-relative speeds
   */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    // By-pass the setpoint generator and drive directly with the desired speeds if necessary
    if (!SwerveConstants.kUseSetpointGenerator) {
      swerveDrive.drive(speeds);
      return;
    }

    // Create a new drive setpoint based on the current drive setpoint & the desired chassis speeds.
    driveSetpoint = setpointGenerator.generateSetpoint(driveSetpoint, speeds, SwerveConstants.kPeriodicTimeSeconds);

    // Move the robot using the new setpoint
    swerveDrive.drive(
      driveSetpoint.robotRelativeSpeeds(),
      driveSetpoint.moduleStates(),
      driveSetpoint.feedforwards().linearForces()
    );
  }

  /**
   * Set each swerve module to brake/coast mode
   * @param brake True to enable motor brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Stop the robot and sets wheel positions to an X formation 
   * to resist being pushed by other robots while on defense.
   */
  private void stopAndLockWheels() {
    driveRobotRelative(new ChassisSpeeds());
    swerveDrive.lockPose();
  }

  /**
   * Resets odometry to a specific pose and resets module encoders.
   * NOTE: Should never need to call this if vision is working properly.
   */
  private void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose == null ? new Pose2d() : pose);
  }

  /**
   * Gets the current heading of the robot from the pose estimator.
   * This is what should be fed into the drive (auto and teleop)
   * functions when calculating chassis speeds & module states.
   * @return Current heading as a Rotation2d
   */
  private Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the current pose of the robot as measured by odometry and vision fusion.
   * @return Current pose 
   */
  private Pose2d getPose() {
    return swerveDrive.getPose();
  }
  
  /**
   * Gets the current robot-relative chassis speeds
   * @return Current ChassisSpeeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Gets the current module states
   * @return Array of current SwerveModuleStates
   */
  private SwerveModuleState[] getModuleStates() {
    return swerveDrive.getStates();
  }

  /**
   * Gets whether field-relative driving is enabled
   * @return Field-relative status
   */
  private boolean isFieldRelative() {
    return fieldRelative && gyro.isConnected();
  }

  /**
   * Gets whether slow mode is enabled
   * @return Slow mode status
   */
  private boolean isSlowMode() {
    return slowMode;
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isFieldRelativeTrigger = new Trigger(this::isFieldRelative)
    .debounce(0.05, DebounceType.kBoth);
    
  public final Trigger isSlowModeTrigger = new Trigger(this::isSlowMode)
    .debounce(0.05, DebounceType.kBoth);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match (autonomous, teleop, post match)
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the drive subsystem at the start of the autonomous phase.
   * Should be called from Robot.autonomousInit() or a command scheduler binding.
   */
  public void autonomousInit() {    
    // Set motors to brake mode for match (do this before resetting encoders)
    setMotorBrake(true);

    // Reset drive setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );
    
    // Reset state variables
    fieldRelative = true;
    slowMode = false;
    acceptedVisionCount = 0;
    rejectedVisionCount = 0;

    // Log initialization
    Utils.logInfo("Drive subsystem initialized for autonomous");
  }

  /**
   * Initializes the drive subsystem at the start of the teleop phase.
   * Should be called from Robot.teleopInit() or a command scheduler binding.
   */
  public void teleopInit() {
    // Set motors to brake mode for match (do this before resetting encoders)
    setMotorBrake(true);

    // Reset drive setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );
    
    // Reset state variables
    fieldRelative = true;
    slowMode = false;
    acceptedVisionCount = 0;
    rejectedVisionCount = 0;
  
    // Log initialization
    Utils.logInfo("Drive subsystem initialized for teleop");
  }

  /**
   * Initializes the drive subsystem for post match (disabled) state.
   * Should be called from Robot.postMatch() or a command scheduler binding.
   */
  public void postMatch() {
    // Ensure brake mode is disabled
    setMotorBrake(false);

    // Reset drive setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );

    // Log initialization
    Utils.logInfo("Drive subsystem initialized for post match");
  }

  /**
   * Add a vision measurement to the pose estimator with validation and filtering.
   * This method can be called by the VisionSubsystem whenever a new vision 
   * measurement is available.
   * @param visionPose The vision pose to add
   * @param timestamp The timestamp of the vision measurement
   * @param stdDevs The standard deviations of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, double[] stdDevs) {
    // Get current time
    double now = Timer.getFPGATimestamp();

    try {      
      // Validate components
      if (visionPose == null || stdDevs == null || stdDevs.length < 3) {
        rejectedVisionCount++;
        return;
      }

      // Reject timestamps older than 0.3 seconds
      if ((now - timestamp) > SwerveConstants.kVisionMeasurementMaxAge) {
        rejectedVisionCount++;
        return;
      }

      // Reject timestamps from the future
      if (timestamp > now) {
        rejectedVisionCount++;
        return;
      }

      // Reject large translation jumps
      double translationDistance = getPose().getTranslation().getDistance(visionPose.getTranslation());
      if (translationDistance > SwerveConstants.kVisionMaxTranslationJumpMeters) {
        rejectedVisionCount++;
        return;
      }

      // If we make it here => add vision measurement to pose estimator
      swerveDrive.addVisionMeasurement(
        visionPose,
        timestamp,
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
      );

      // Increment the count of accepted vision measurements for monitoring purposes
      acceptedVisionCount++;
    } catch (Exception e) {
      Utils.logError("Error adding vision measurement: " + e.getMessage());
    }
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  /**
   * Command to characterize the robot drive motors using SysId
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new Config(),
        this, 
        swerveDrive, 
        12, 
        true
      ),
      3.0, 
      5.0, 
      3.0
    );
  }

  /**
   * Command to characterize the robot angle motors using SysId
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(),
        this, 
        swerveDrive
        ),
      3.0, 
      5.0, 
      3.0
    );
  }

  // ---------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ---------------------------------------------------------------------------------------

  /**
   * Drive the robot using the raw joystick inputs.
   * This function applies deadband, input shaping, input squaring and slow mode scaling. 
   * Joystick inputs are converted to robot chassis speeds in m/s and rad/s.
   * Field relative driving is automatically disabled if the gyro is disconnected. 
   * @param xSpeedSupplier Speed in x direction (-1 to 1)
   * @param ySpeedSupplier Speed in y direction (-1 to 1)
   * @param rSpeedSupplier Rotation speed (-1 to 1)
   */
  public Command driveCommand(
    DoubleSupplier xSpeedSupplier, 
    DoubleSupplier ySpeedSupplier, 
    DoubleSupplier rSpeedSupplier
  ) {
    // Return a command that runs the drive logic
    return run(() -> {
      // Get raw joystick inputs from the suppliers every cycle
      double rawX = xSpeedSupplier.getAsDouble();
      double rawY = ySpeedSupplier.getAsDouble();
      double rawR = rSpeedSupplier.getAsDouble();
      
      double magnitude = Math.hypot(rawX, rawY);
      double xSpeed = 0;
      double ySpeed = 0;

      // Apply circular deadband for translation 
      if (magnitude > SwerveConstants.kJoystickDeadband) {
        // Rescale so speed starts at 0 at the edge of the deadband
        double clippedMagnitude = (magnitude - SwerveConstants.kJoystickDeadband) / (1.0 - SwerveConstants.kJoystickDeadband);
        
        // Apply squaring/cubing to the clipped magnitude
        double curvedMagnitude = Math.copySign(Math.pow(clippedMagnitude, SwerveConstants.kJoystickSmoothing), clippedMagnitude);

        // Re-apply the direction sign to the new magnitude
        xSpeed = (rawX / magnitude) * curvedMagnitude;
        ySpeed = (rawY / magnitude) * curvedMagnitude;
      }

      // Apply 1D deadband and squaring for rotation
      double rSpeed = MathUtil.applyDeadband(rawR, SwerveConstants.kJoystickDeadband);
      rSpeed = Math.copySign(Math.pow(rSpeed, SwerveConstants.kJoystickSmoothing), rSpeed);

      // If slow mode is enabled, scale down speeds for finer control
      if (isSlowMode()) {
        xSpeed *= SwerveConstants.kSlowModeScaling;
        ySpeed *= SwerveConstants.kSlowModeScaling;
        rSpeed *= SwerveConstants.kSlowModeScaling;
      }

      // Convert the joystick's -1..1 to m/s and rad/s
      double xSpeedMPS = xSpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedMPS = ySpeed * SwerveConstants.kMaxSpeedMetersPerSecond;
      double rSpeedRad = rSpeed * SwerveConstants.kMaxAngularSpeedRadsPerSecond;

      // Create robot-relative chassis speeds
      ChassisSpeeds chassisSpeeds;
      if (isFieldRelative()) {
        // Invert translation inputs for red alliance so that pushing the joystick
        // forward always moves the robot away from the driver station
        int invert = Utils.isRedAlliance() ? -1 : 1;

        // Convert field-relative speeds to robot-relative speeds
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeedMPS * invert, 
          ySpeedMPS * invert, 
          rSpeedRad, 
          getHeading()
        );
      } else {
        // Use robot-relative speeds directly
        chassisSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rSpeedRad);
      }

      // Drive the robot with robot-relative speeds
      driveRobotRelative(chassisSpeeds);
    });
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner
   * @param targetPose The Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command driveToPoseCommand(Supplier<Pose2d> targetSupplier) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Get the target pose from the supplier when the command is scheduled
      Pose2d targetPose = targetSupplier.get();

      // Check for null target pose
      if (targetPose == null) {
        return Commands.print("driveToPoseCommand: Target pose is null. Cannot generate pathfinding command.");
      }

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(targetPose, SwerveConstants.kPathfindingConstraints, 0.0);
    }, Set.of(this));
  }

  /**
   * Creates a command to drive the robot a given distance in a given direction.
   * @param distanceMeters The distance to drive in meters
   * @param heading The field-relative heading to drive in (e.g. Rotation2d.fromDegrees(0) = forward)
   * @return Command to drive the given distance
   */
  public Command driveDistanceCommand(double distanceMeters, Rotation2d heading) {
    return Commands.defer(() -> {
      // Calculate the target pose based on current pose + offset in the given direction
      Pose2d currentPose = getPose();

      // Calculate the translation offset based on the desired distance and heading
      Translation2d offset = new Translation2d(distanceMeters, heading);

      // Calculate the target pose by applying the offset to the current pose
      Pose2d targetPose = new Pose2d(
        currentPose.getTranslation().plus(offset),
        currentPose.getRotation() // Maintain current heading
      );

      // Return a command to drive to the target pose
      return driveToPoseCommand(() -> targetPose);
    }, Set.of(this));
  }

  /**
   * Creates a command to align the robot at a specified AprilTag
   * @param tagId The fiducial ID of the AprilTag to align with
   * @param xOffset The X offset from the tag in meters (positive is forward, negative is backward)
   * @param yOffset The Y offset from the tag in meters (positive is left, negative is right)
   * @return Command to align with the AprilTag
   */  
  public Command alignToTagCommand(int tagId, double xOffset, double yOffset) {
    return driveToPoseCommand(() -> {
      // Get the pose of the AprilTag from the field layout
      var tagPose = FieldConstants.kFieldLayout.getTagPose(tagId);

      // Calculate offset
      if (tagPose.isPresent()) {
        // Option 1
        return tagPose.get().toPose2d().transformBy(
          new Transform2d(xOffset, yOffset, Rotation2d.fromDegrees(180))
        );

        // Option 2
        // return tagPose.get().toPose2d().transformBy(
        //   new Transform2d(
        //     new Translation2d(xOffset, yOffset), 
        //     tagPose.get().toPose2d().getRotation().plus(Rotation2d.fromDegrees(180))
        //   )
        // );
      }

       // driveToPoseCommand will handle this null safely
      return null;
    });
  }

  /**
   * Reset the odometry to origin (0,0,0) with module encoder reset.
   * NOTE: Should never need to call this if vision is working properly.
   */
  public Command resetOdometryCommand() {
    return resetOdometryCommand(null);
  }

  /**
   * Reset the odometry to a known pose.
   */
  public Command resetOdometryCommand(Pose2d pose) {
    return runOnce(() -> resetOdometry(pose))
      .ignoringDisable(true)
      .withName("Drive_ResetOdometry");
  }

  /**
   * Stop the robot and set the wheels to an X formation
   */
  public Command stopAndLockWheelsCommand() {
    return runOnce(this::stopAndLockWheels)
      .withName("Drive_StopAndLockWheels");
  }

  /**
   * Enables/disables field-relative driving mode
   */
  public Command toggleFieldRelativeModeCommand() {
    return runOnce(() -> this.fieldRelative = !this.fieldRelative)
      .ignoringDisable(true)
      .withName("Drive_ToggleFieldRelativeMode");
  }

  /**
   * Enable slow mode
   */
  public Command enableSlowModeCommand() {
    return runOnce(() -> this.slowMode = true)
      .ignoringDisable(true)
      .withName("Drive_EnableSlowMode");
  }

  /**
   * Disable slow mode
   */
  public Command disableSlowModeCommand() {
    return runOnce(() -> this.slowMode = false)
      .ignoringDisable(true)
      .withName("Drive_DisableSlowMode");
  }

  /**
   * Updates SmartDashboard with subsystem telemetry
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Field Relative", this::isFieldRelative, null);
    builder.addBooleanProperty("Slow Mode", this::isSlowMode, null);
    builder.addDoubleProperty("Accepted Vision Count", () -> Utils.showDouble(acceptedVisionCount), null);
    builder.addDoubleProperty("Rejected Vision Count", () -> Utils.showDouble(rejectedVisionCount), null);
  }
}
