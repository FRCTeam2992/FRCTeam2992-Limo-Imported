package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class TwoBallPath extends SwerveTrajectoryGenerator {

    public TwoBallPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.twoBallTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
    
        //addHeadingWaypoint(0.1, startRotation);
        addHeadingWaypoint(0.1, 226.5);
        addHeadingWaypoint(1, 226.5);
    }
}
