package frc.lib.drive.swerve.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;



public class SwerveTrajectoryGenerator {

    // Trajectory Settings
    private double maxVelocity;
    private double maxAcceleration;
    private double startRotation = 0.0;

    // Waypoints
    private Pose2d startPose;
    private Pose2d endPose;
    private List<Translation2d> interiorWaypoints;
    private List<TrajectoryAngleState> headingWaypoints;

    // Set Trajectory
    private Trajectory setTrajectory;

    public SwerveTrajectoryGenerator(Pose2d startPose, Pose2d endPose, double maxVelocityMetersPerSecond,
            double maxAccelerationMetersPerSecondSq) {
        // Save the Start and End Translations
        this.startPose = startPose;
        this.endPose = endPose;

        // Save the Trajectory Settings
        this.maxVelocity = maxVelocityMetersPerSecond;
        this.maxAcceleration = maxAccelerationMetersPerSecondSq;

        // Initialize the Interior Waypoints List
        interiorWaypoints = new ArrayList<>();

        // Initialize the Angle Waypoints Map
        headingWaypoints = new ArrayList<>();
    }

    public SwerveTrajectoryGenerator(Trajectory trajectory) {
        // Save the Set Trajectory
        setTrajectory = trajectory;

        // Initialize the Angle Waypoints Map
        headingWaypoints = new ArrayList<>();
    }

    public SwerveTrajectoryGenerator() {
        this(new Pose2d(), new Pose2d(), 0.0, 0.0);
    }

    public void setStartPose(Pose2d pose) {
        startPose = pose;
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public void setEndPose(Pose2d pose) {
        endPose = pose;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

    public void setMaxVelocity(double metersPerSecond) {
        maxVelocity = metersPerSecond;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxAcceleration(double metersPerSecondSquared) {
        maxAcceleration = metersPerSecondSquared;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setStartRotation(double degrees) {
        startRotation = degrees;
    }

    public double getStartRotation() {
        return startRotation;
    }

    public void addWaypoint(double x, double y) {
        interiorWaypoints.add(new Translation2d(x, y));
    }

    public void addHeadingWaypoint(double seconds, double degrees) {
        headingWaypoints.add(new TrajectoryAngleState(seconds, -degrees));
    }

    public void addTimedHeadingWaypoint(double startTime, double endTime, double degrees) {
        headingWaypoints.add(new TrajectoryAngleState(startTime, endTime, -degrees));
    }

    public void setTrajectory(Trajectory trajectory) {
        setTrajectory = trajectory;
    }

    public SwerveTrajectory generateSwerveTrajectory() {
        Trajectory trajectory;

        // Check for Set Trajectory
        if (setTrajectory != null) {
            trajectory = setTrajectory;
        } else {
            // Create the Trajectory Config
            TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);

            // Generate the Trajectory
            trajectory = TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, config);
        }

        // Sort the Heading Waypoints
        Collections.sort(headingWaypoints);

        // Create a Temporary Heading List
        List<TrajectoryAngleState> tempHeadingList = new ArrayList<>();

        // Process Each Heading
        for (TrajectoryAngleState tempState : headingWaypoints) {
            // Check for End Time
            if (tempState.getEndTime() > 0.0) {
                // Step Time State
                double stepTime = 0.0;

                // Get the Last Set Angle and the Target Set Angle
                double lastAngle = tempHeadingList.get(tempHeadingList.size() - 1).getAngle();
                double targetAngle = tempState.getAngle();

                // Get the Angle Change
                double deltaAngle = targetAngle - lastAngle;

                // Normalize the Delta Angle (-180 - 180)
                if (deltaAngle > 180.0) {
                    deltaAngle -= 360.0;
                } else if (deltaAngle < -180.0) {
                    deltaAngle += 360.0;
                }

                // Get the Angle Steps per Second
                double angleSteps = deltaAngle / (tempState.getEndTime() - tempState.getTime());

                // Interpolate the Angle for every 20 Milliseconds
                while (stepTime <= (tempState.getEndTime() - tempState.getTime())) {
                    // Caculate the Step Target Angle
                    double tempAngle = lastAngle + (angleSteps * stepTime);

                    // Normalize the Step Target Angle (-180 - 180)
                    if (tempAngle > 180.0) {
                        tempAngle -= 360.0;
                    } else if (tempAngle < -180.0) {
                        tempAngle += 360.0;
                    }

                    // Add the Step Target Angle to the Heading List
                    tempHeadingList.add(new TrajectoryAngleState(tempState.getTime() + stepTime, tempAngle));

                    // Increase the Step Time State
                    stepTime += 0.02;
                }
            } else {
                // Add the Angle State to the Heading List
                tempHeadingList.add(tempState);
            }
        }

        return new SwerveTrajectory(trajectory, tempHeadingList, startRotation);
    }
}
