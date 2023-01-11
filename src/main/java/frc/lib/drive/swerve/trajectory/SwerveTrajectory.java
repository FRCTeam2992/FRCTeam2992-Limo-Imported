package frc.lib.drive.swerve.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;


public class SwerveTrajectory {

    // Variables
    private Trajectory trajectory;
    private List<TrajectoryAngleState> headingWaypoints;
    private double startRotation;

    public SwerveTrajectory(Trajectory trajectory, List<TrajectoryAngleState> headingWaypoints, double startRotation) {
        // Save the Variables
        this.trajectory = trajectory;
        this.headingWaypoints = headingWaypoints;
        this.startRotation = startRotation;

        // Sort the Heading Waypoint List
        Collections.sort(headingWaypoints);
    }

    public SwerveTrajectory(Trajectory trajectory, List<TrajectoryAngleState> headingWaypoints) {
        this(trajectory, headingWaypoints, 0.0);
    }

    public SwerveTrajectory() {
        this(new Trajectory(), new ArrayList<TrajectoryAngleState>(), 0.0);
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public void setHeadingWaypoints(List<TrajectoryAngleState> headingWaypoints) {
        // Sort the Heading Waypoint List
        Collections.sort(headingWaypoints);

        this.headingWaypoints = headingWaypoints;
    }

    public List<TrajectoryAngleState> getHeadingWavpoints() {
        return headingWaypoints;
    }

    public void setStartRotation(double degrees) {
        startRotation = degrees;
    }

    public double getStartRotation() {
        return startRotation;
    }

    public double getDesiredHeading(double time) {
        TrajectoryAngleState desiredHeadingState = new TrajectoryAngleState(0.0, 0.0);

        for (TrajectoryAngleState tempState : headingWaypoints) {
            if (time >= tempState.getTime()) {
                desiredHeadingState = tempState;
            }
        }

        return desiredHeadingState.getAngle();
    }
}
