package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class FiveBallFinalPart2Path extends SwerveTrajectoryGenerator {

    public FiveBallFinalPart2Path(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallFinalPart2Trajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        addHeadingWaypoint(0.001, startRotation);
       
        addHeadingWaypoint(3.00, 135);
    }
}
