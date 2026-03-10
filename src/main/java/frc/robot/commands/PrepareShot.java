package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class PrepareShot extends Command   {

    ShooterSubsystem shooter;
    HoodSubsystem hood;
    private final Supplier<Pose2d> robotPoseSupplier;

    public PrepareShot(ShooterSubsystem shooter, HoodSubsystem hood, Supplier<Pose2d> robotPoseSupplier) {
        addRequirements(shooter, hood);
        this.shooter = shooter;
        this.hood = hood;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Distance getDistanceToHub() {
        final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        final Translation2d hubPosition;
        if (alliance.isPresent()   && alliance.get() == Alliance.Blu e ) {
            hubPosition = new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return Meters.of(robotPosition.getDistance(hubPosition));
    } 
 

    
    
    public double rpm;
    public double pivotAngle;

    public ShooterState(double rpm, double pivotAngle) {
        this.rpm = rpm;
        this.pivotAngle = pivotAngle;
    }

    @Override
    public ShooterState interpolate(ShooterState endValue, double t) {
        // 't' is a value from 0 to 1 representing how far we are between points
        double lerpRPM = this.rpm + (endValue.rpm - this.rpm) * t;
        double lerpPivot = this.pivotAngle + (endValue.pivotAngle - this.pivotAngle) * t;
        
        return new ShooterState(lerpRPM, lerpPivot);
    }



    private ShooterState getShooterState(){
        Distance hubDistance = getDistanceToHub();

        // Distance (meters) -> ShooterState (RPM & Angle)
        InterpolatingTreeMap<Double, ShooterState> shooterTable = new InterpolatingTreeMap<>();

        // Add your calibration data
        // Distance, new ShooterState(RPM, PivotPosition)
        shooterTable.put(1.0, new ShooterState(1500, 10.0));
        shooterTable.put(3.0, new ShooterState(2500, 25.0));

        // To use it:
        double distanceToTarget = 2.0; // We are halfway between our points
        ShooterState currentSetpoints = shooterTable.get(distanceToTarget);

        return currentSetpoints;
    }

}
