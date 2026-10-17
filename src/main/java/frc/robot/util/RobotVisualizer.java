package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * A simple visualizer to combine all the robot's mechanisms in one view on the SmartDashboard.
 * This is just a static utility class; it doesn't need to be instantiated.
 */
public final class RobotVisualizer {
  private RobotVisualizer() {}

  // Canvas in meters: 1.5 wide, tall enough for max elevator height + arm
  private static final Mechanism2d mech = new Mechanism2d(1.5, 2.5);

  // Everything hangs off one root at the robot's base
  private static final MechanismRoot2d root = mech.getRoot("Robot", 0.75, 0.1);

  /**
   * Registers the Mechanism2d object with the SmartDashboard so it can be displayed. 
   * This is done in a static block so it happens when the class is loaded, and only once.
   */
  static {
    SmartDashboard.putData("Robot Mech2d", mech);
  }

  // ==========================================================================================
  // Private subsystems mechansim 2d objects for the visualize
  // ==========================================================================================

  // Elevator points straight up (90 degrees)
  private static final MechanismLigament2d elevator = root.append(
    new MechanismLigament2d("Elevator", 0.0, 90, 6, new Color8Bit(Color.kOrange)));

  // Arm is appended to the END of the elevator ligament, so it rides up and down with it.
  // Ligament angles are relative to the parent, so this starts at -90 (= horizontal in the world).
  private static final MechanismLigament2d arm = elevator.append(
    new MechanismLigament2d("Arm", 0.4, -90, 4, new Color8Bit(Color.kGreen)));

  // Intake gets its own root at the front of the chassis, independent of the elevator
  private static final MechanismRoot2d intakeRoot = mech.getRoot("Intake", 1.1, 0.2);
  private static final MechanismLigament2d intake = intakeRoot.append(
    new MechanismLigament2d("Intake", 0.3, 0, 4, new Color8Bit(Color.kBlue)));

  // ==========================================================================================
  // Public API for other subsystems to update the visualizer
  // ==========================================================================================

  /**
   * Sets the elevator height in the visualizer. The height is specified in meters, and the method
   * sets this length directly on the elevator ligament, which is independent of the arm.
   * @param meters Elevator height in meters
   */
  public static void setElevatorHeight(double meters) {
    elevator.setLength(meters);
  }

  /**
   * Sets the arm angle in the visualizer. The angle is specified in the world frame, 
   * where 0 degrees is horizontal forward and +90 degrees is straight up. The 
   * method converts this angle to be relative to the elevator's orientation 
   * before setting it on the arm ligament.
   * @param angle Arm angle in the world frame: 0 = horizontal forward, +90 = straight up
   */
  public static void setArmAngle(Rotation2d angle) {
    arm.setAngle(angle.getDegrees() - 90.0);  // convert to relative-to-elevator
  }

  /**
   * Sets the intake angle in the visualizer. The angle is specified in the world frame,
   * where 0 degrees is horizontal forward and negative angles are downward. The method
   * sets this angle directly on the intake ligament, which is independent of the elevator.
   * @param angle Intake angle in the world frame: 0 = horizontal forward, negative = downward
   */
  public static void setIntakeAngle(Rotation2d angle) {
    intake.setAngle(angle.getDegrees());
  }
}
