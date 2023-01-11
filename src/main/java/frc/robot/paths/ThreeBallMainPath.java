package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class ThreeBallMainPath extends SwerveTrajectoryGenerator {

    public ThreeBallMainPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.threeBallMainTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        addHeadingWaypoint(.2, 90);
        addHeadingWaypoint(.6, -130.0);
    }
}
