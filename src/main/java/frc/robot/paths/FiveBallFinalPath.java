package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class FiveBallFinalPath extends SwerveTrajectoryGenerator {

    public FiveBallFinalPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallFinalTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        addHeadingWaypoint(0.001, startRotation);
        addTimedHeadingWaypoint(0.1, 1.0, 135);
        // addHeadingWaypoint(0.1, startRotation);
        addHeadingWaypoint(3.00, 135);
    }
}
