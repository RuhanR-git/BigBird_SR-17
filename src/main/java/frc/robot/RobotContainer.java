package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final SwerveSubsystem driveBase = new SwerveSubsystem();

  // Driver Controller on Port 0
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // SETUP THE DRIVE STRATEGY
    // This creates a "Stream" of inputs from your joystick to the robot
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        driveBase.getSwerveDrive(),
        // LEFT STICK Y: Moves robot Forward/Backward
        () -> m_driverController.getLeftY() * 1, 
        // LEFT STICK X: Moves robot Left/Right (Strafing)
        () -> m_driverController.getLeftX() * 1) 
        // RIGHT STICK X: "Steering" - Makes wheels form a circle to rotate the robot
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8) // 80% speed for safety, change to 1.0 for full power
        .allianceRelativeControl(true); // Field-Oriented: "Forward" is always away from you

    // Apply the drive command as the default
    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveInputStream));
  }

  private void configureBindings() {
    // DIAGRAM HELP: If the robot's "Forward" gets confused, press the 'Y' button
    // to reset the gyro so "Forward" is wherever the robot is currently facing.
    m_driverController.y().onTrue(driveBase.zeroGyroCommand());
    
    // Safety: Press 'B' to lock wheels in an 'X' to stop movement
    m_driverController.b().whileTrue(driveBase.lockPoseCommand());
  }

  public Command getAutonomousCommand() {
    return null; // Add auto logic here later
  }
}