// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Autos;
import frc.robot.commands.Feedback;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.rumble.RumbleSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  // Initialize our controllers
  private final CommandXboxController driverXbox = new CommandXboxController(0);
   
  // The robot's subsystems are defined here...
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final RumbleSubsystem rumbleSubsystem = new RumbleSubsystem(driverXbox);
  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem::addVisionMeasurement);
 
  // Initalize command factories
  private final Autos autos = new Autos(driveSubsystem, ledSubsystem, visionSubsystem, climberSubsystem);
  private final Feedback feedback = new Feedback(ledSubsystem, rumbleSubsystem);

  // Auto choosers
  private final SendableChooser<Command> delayChooser = new SendableChooser<>();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Track match state
  private boolean wasInAuto = false;
  private boolean wasInTeleop = false;

  public RobotContainer() {
    // set our default driving method
    driveSubsystem.setDefaultCommand(driveSubsystem.driveCommand(
      () -> -1 * driverXbox.getLeftY(),
      () -> -1 * driverXbox.getLeftX(),
      () -> -1 * driverXbox.getRightX()
    ));

    // register named autos
    registerNamedAutos();

    // configure auto routines
    configureAutos();

    // configure controller button bindings
    configureButtonBindings();

    // configure state event triggers
    configureEventTriggers();
   
    // silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Register named commands before the creation of any PathPlanner Autos or Paths. 
   * It is recommended to do this in RobotContainer, after subsystem 
   * initialization, but before the creation of any other commands.
   */
  private void registerNamedAutos() {
    NamedCommands.registerCommand("DRIVE_ALIGN_TO_TARGET", Commands.none());
  }

  /**
   * Configure the autonomous command chooser and delay chooser and add them to the dashboard.
   * This reads and warms up all the autos specified in the local array.
   */
  private void configureAutos() {
    // Explicitly add autos to the autoChooser. These can be simple autos that are
    // defined in our Autos.java or they can be PathPlanner autos located in the 
    // "/deploy/pathplanner/autos" directory. Use the auto name without the 
    // ".auto" extension for the second argument.
    autoChooser.setDefaultOption("No auto", Commands.none());
    autoChooser.addOption("Example", autos.exampleAutoRoutine());
    autoChooser.addOption("Two Piece Auto", AutoBuilder.buildAuto("TwoPieceAuto"));
    
    // Add auto chooser to dashboard
    SmartDashboard.putData("Auto Command", autoChooser);

    // Configure the available auto delay options
    delayChooser.setDefaultOption("No delay", Commands.none());
    delayChooser.addOption("1.0 second", Commands.waitSeconds(1.0));
    delayChooser.addOption("1.5 seconds", Commands.waitSeconds(1.5));
    delayChooser.addOption("2.0 seconds", Commands.waitSeconds(2.0));
    delayChooser.addOption("2.5 seconds", Commands.waitSeconds(2.5));
    delayChooser.addOption("3.0 seconds", Commands.waitSeconds(3.0));
    delayChooser.addOption("3.5 seconds", Commands.waitSeconds(3.5));
    delayChooser.addOption("4.0 seconds", Commands.waitSeconds(4.0));
    delayChooser.addOption("4.5 seconds", Commands.waitSeconds(4.5));
    delayChooser.addOption("5.0 seconds", Commands.waitSeconds(5.0));
    
    // Add delay chooser to dashboard
    SmartDashboard.putData("Auto Delay", delayChooser);
  }

  /**
   * Use this method to define your button->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureButtonBindings() {
    // manually reset odometry & climber home position
    driverXbox.start().onTrue(Commands.parallel(
      driveSubsystem.resetOdometryCommand()
    ));

    // toggles the drive mode: field-relative vs robot-relative
    driverXbox.back().onTrue(driveSubsystem.toggleFieldRelativeModeCommand());

    // teleop button bindings
    RobotModeTriggers.teleop().and(driverXbox.a()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.b()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.x()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.y()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.leftTrigger()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.leftBumper()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.rightTrigger()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.rightBumper()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.povUp()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.povRight()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.povDown()).onTrue(Commands.none());
    RobotModeTriggers.teleop().and(driverXbox.povLeft()).onTrue(Commands.none());

    // show various feedbacks for fun when disabled
    RobotModeTriggers.disabled().and(driverXbox.a()).onTrue(ledSubsystem.flameCommand());
    RobotModeTriggers.disabled().and(driverXbox.b()).onTrue(ledSubsystem.knightRiderCommand());
    RobotModeTriggers.disabled().and(driverXbox.x()).onTrue(ledSubsystem.heartbeatCommand());
    RobotModeTriggers.disabled().and(driverXbox.y()).onTrue(ledSubsystem.discoCommand());
    RobotModeTriggers.disabled().and(driverXbox.povUp()).onTrue(ledSubsystem.teamColorsCommand());
    RobotModeTriggers.disabled().and(driverXbox.povRight()).onTrue(ledSubsystem.meteorCommand());
    RobotModeTriggers.disabled().and(driverXbox.povDown()).onTrue(ledSubsystem.lavaLampCommand());
    RobotModeTriggers.disabled().and(driverXbox.povLeft()).onTrue(ledSubsystem.offCommand());

    // SysId bindings - typically on a controller with a modifier button held
    RobotModeTriggers.test().and(driverXbox.a()).whileTrue(driveSubsystem.sysIdDriveMotorCommand());
    RobotModeTriggers.test().and(driverXbox.b()).whileTrue(driveSubsystem.sysIdAngleMotorCommand());
  }

  /**
   * Use this to configure triggers that should respond to robot/subsystem state changes.
   */
  private void configureEventTriggers() {
    // feedback when drive has switched to robot-relative mode
    RobotModeTriggers.teleop()
      .and(driveSubsystem.isFieldRelativeTrigger)
      .onFalse(feedback.warningCommand());
  }

  /**
   * Use this to pass the autonomous command to the main Robot class. The selected command will 
   * be automatically scheduled on autonomous start by the main Robot class. You can add 
   * additional auto options by adding them to the auto chooser.
   * @return the command to run in autonomous, or Commands.none() to run nothing
   */
  public Command getAutonomousCommand() {
    return delayChooser.getSelected().andThen(autoChooser.getSelected());
  }

  /**
   * Initializes all subsystems for autonomous mode. Should be called from Robot.autonomousInit().
   */
  public void autonomousInit() {
    // Track state change
    wasInAuto = true;
    wasInTeleop = false;

    // Initialze subsystems
    driveSubsystem.autonomousInit();
  }

  /**
   * Initializes all subsystems for teleop mode. Should be called from Robot.teleopInit().
   */
  public void teleopInit() {
    // Track state change
    wasInTeleop = true;

    // When testing we sometimes enable the robot in teleop and skip autoonomous.
    // If that is the case, init autonomous instead of teleop.
    if (!wasInAuto) {
      driveSubsystem.autonomousInit();
    } else {
      driveSubsystem.teleopInit();
    }
  }

  /**
   * Initializes all subsystems for post match mode. Should be called from Robot.disabledInit().
   */
  public void postMatch() {
    // Clean up post match
    if (wasInTeleop) {
      wasInAuto = false;
      wasInTeleop = false;
      driveSubsystem.postMatch();
    }
  }
}
