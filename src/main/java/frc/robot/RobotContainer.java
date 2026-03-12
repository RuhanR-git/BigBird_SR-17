package frc.robot;

// WPILIB and YAGSL imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

// Imports from our code
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem driveBase = new SwerveSubsystem();

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    // Note: Removed redundant NamedCommands.registerCommand here to prevent 
    // circular logic since you are calling the full auto file directly.

    configureBindings();
  }

  private void configureBindings() 
  {
    // --- Driver bindings for driving (Driver controller)---

    //Left joystick: Strafing
    //Right joystick: Rotation
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        driveBase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * 1, 
        () -> m_driverController.getLeftX() * 1) 
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) 
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    // --- Extra Driver bindings for driving ---

    // Y -> Zero Gyro
    // B -> Lock Pose, locks robot wheels to only move forward
    driveBase.setDefaultCommand(driveBase.driveFieldOriented(driveInputStream));
    m_driverController.y().onTrue(driveBase.zeroGyroCommand());
    m_driverController.b().whileTrue(driveBase.lockPoseCommand());

    // --- Operator bindings for intake (Operator controller)---

    // Left Trigger -> Intake Balls
    m_operatorController.leftTrigger()
    .whileTrue(m_intakeSubsystem.intakeCommand()
    .alongWith(Commands.print("Intaking balls...")))
    .onFalse(m_intakeSubsystem.stowCommand());
    
    // Right Trigger -> Jiggle Balls towards shooter
    m_operatorController.rightTrigger()
      .whileTrue(m_intakeSubsystem.shooterJiggleCommand()
      .alongWith(Commands.print("Jiggling intake...")))
      .onFalse(m_intakeSubsystem.stowCommand());
  }

  /**
   * Returns the hard-coded autonomous command.
   */
  public Command getAutonomousCommand() {
    return driveBase.getTestAutoCommand();
  }
}