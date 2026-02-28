package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();

  public SwerveSubsystem() { 
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    // Publish the field to NetworkTables for Glass visualization
    SmartDashboard.putData("Field", m_field);

    // Setup AutoBuilder
    setupPathPlanner();
  }

  public final void setupPathPlanner() {
    try {
        RobotConfig config = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
            this::getPose, 
            swerveDrive::resetOdometry, 
            swerveDrive::getRobotVelocity, 
            (speeds, feedforwards) -> swerveDrive.drive(speeds), 
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    } catch (IOException | ParseException e) {
        DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
    }
}

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public Command zeroGyroCommand() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  public Command lockPoseCommand() {
    return run(() -> swerveDrive.lockPose());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // public void registerNamedCommands() {
  //   // Example: Map the string "Intake" in the GUI to a command
  //   // NamedCommands.registerCommand("Intake", intakeSubsystem.intakeCommand());
    
  //   // You can also use print commands for testing
  //   NamedCommands.registerCommand("Marker1", Commands.print("Passed Marker 1"));
  // }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    m_field.setRobotPose(swerveDrive.getPose());
  }
}