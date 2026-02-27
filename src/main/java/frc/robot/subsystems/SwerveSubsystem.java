package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive swerveDrive;

  public SwerveSubsystem() { 
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      // Initialize the drive base using your JSON files
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  // The command used in RobotContainer to actually move the wheels
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  // Resets the Gyro (Essential for Field Oriented Driving)
  public Command zeroGyroCommand() {
    return runOnce(() -> swerveDrive.zeroGyro());
  }

  // Forces wheels into an 'X' shape so the robot can't be pushed
  public Command lockPoseCommand() {
    return run(() -> swerveDrive.lockPose());
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // YAGSL handles odometry updates internally
  }
}