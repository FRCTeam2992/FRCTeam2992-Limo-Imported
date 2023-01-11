package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class FiveBallFinalPart1Path extends SwerveTrajectoryGenerator {

    public FiveBallFinalPart1Path(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallFinalPart1Trajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        addHeadingWaypoint(0.001, startRotation);
        addTimedHeadingWaypoint(0.1, 1.0, 135);
        // addHeadingWaypoint(0.1, startRotation);
        addHeadingWaypoint(3.00, 135);
    }
}
