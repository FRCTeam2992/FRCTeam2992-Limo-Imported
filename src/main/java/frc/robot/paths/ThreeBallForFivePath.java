package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class ThreeBallForFivePath extends SwerveTrajectoryGenerator {

    public ThreeBallForFivePath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.threeBallForFiveTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        addHeadingWaypoint(0.0005, startRotation);
        addTimedHeadingWaypoint(0.001, 0.2, 90);
        addTimedHeadingWaypoint(0.65, 0.9, 235);
        
    }
}
