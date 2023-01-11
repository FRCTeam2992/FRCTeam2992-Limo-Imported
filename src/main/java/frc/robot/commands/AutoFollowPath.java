// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.swerve.trajectory.SwerveTrajectory;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoFollowPath extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;

  // Trajectory Instance
  private SwerveTrajectory mSwerveyTrajectory;
  private Trajectory mTrajectory;

  // Drive Controller Instance
  private HolonomicDriveController controller;

  // Timer
  private Timer elapsedTimer;

  // Reset Odometry
  private boolean mResetOdometry;

  //Gyro Offset
  private boolean mSetGyroOffset;
  private double mGyroOffset;
  private DataLog mDataLog;
  private DoubleLogEntry gyroReadLog;
  private DoubleLogEntry trajectoryHeadingReadLog;
  private DoubleLogEntry trajectoryTimeStamp;
  private DoubleLogEntry omegaRotation;

  private ProfiledPIDController thetaController;

  public AutoFollowPath(Drivetrain subsystem, SwerveTrajectory swerveTrajectory, boolean resetOdometry,
        boolean setGyroOffset, double gyroOffset) {
    // Subsystem Instance
    mDriveTrain = subsystem;
    mResetOdometry = resetOdometry;
    mSetGyroOffset = setGyroOffset;
    mGyroOffset = gyroOffset;
    

    // Set the Subsystem Requirement
    addRequirements(mDriveTrain);

    // Save the Swerve Trajectory
    mSwerveyTrajectory = swerveTrajectory;

    // Get the Trajectory
    mTrajectory = mSwerveyTrajectory.getTrajectory();
    mTrajectory.transformBy(new Transform2d(new Translation2d(Constants.goalX, Constants.goalY), Rotation2d.fromDegrees(0.0)));

    // Create the Theta Controller
    thetaController = new ProfiledPIDController(Constants.thetaCorrectionP,
        Constants.thetaCorrectionI, Constants.thetaCorrectionD,
        new TrapezoidProfile.Constraints(Constants.maxThetaVelocity, Constants.maxThetaAcceleration));

    // Enable Wrapping on the Theta Controller (Radians)
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize the Drive Controller
    controller = new HolonomicDriveController(
        new PIDController(Constants.xCorrectionP, Constants.xCorrectionI, Constants.xCorrectionD),
        new PIDController(Constants.yCorrectionP, Constants.yCorrectionI, Constants.yCorrectionD), thetaController);

    // Timer
    elapsedTimer = new Timer();

    if (Constants.dataLogging){
      mDataLog = DataLogManager.getLog();
      gyroReadLog = new DoubleLogEntry(mDataLog, "/afp/odometryHeading");
      trajectoryHeadingReadLog = new DoubleLogEntry(mDataLog, "/afp/trajectoryHeading");
      trajectoryTimeStamp = new DoubleLogEntry(mDataLog, "/afp/trajectoryTimeStamp");
      omegaRotation = new DoubleLogEntry(mDataLog, "/afp/omegaRotation");
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the Trajectory Start Pose
    Pose2d trajectoryStartPose = mTrajectory.getInitialPose();

    // Do we need to reset gyro offset for defined starting position
    if (mSetGyroOffset) {
      mDriveTrain.gyroOffset = mGyroOffset;
    }

    // Set the Odometry Position to the Trajectory Start Position
    if (mResetOdometry) {
      mDriveTrain.setOdometryPosition(true, new Pose2d(trajectoryStartPose.getX(), trajectoryStartPose.getY(),
      Rotation2d.fromDegrees(-mDriveTrain.getGyroYaw())));
    }

    thetaController.reset(mDriveTrain.latestSwervePose.getRotation().getRadians(), 0.0);

    // Reset and Start the Elapsed Timer
    elapsedTimer.reset();
    elapsedTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the Current Time
    double currentTime = elapsedTimer.get();

    // Get the Latest State
    Trajectory.State latestState = mTrajectory.sample(currentTime);
    // SmartDashboard.putNumber("Trajectory X", latestState.poseMeters.getX() * 100.0 / 2.54);
    // SmartDashboard.putNumber("Trajectory Y", latestState.poseMeters.getY() * 100.0 / 2.54);
    

    // Get the Desired Heading
    double heading = mSwerveyTrajectory.getDesiredHeading(currentTime);
    // while (heading > Math.PI) {
    //   heading -= 2 * Math.PI;
    // }
    // while (heading <= -1 * Math.PI) {
    //   heading += 2 * Math.PI;
    // }
    // SmartDashboard.putNumber("trajectory heading", heading);


    // Get the Ajusted Speeds
    ChassisSpeeds adjustSpeeds = controller.calculate(mDriveTrain.latestSwervePose, latestState,
        Rotation2d.fromDegrees(heading));


    // Data Logging
    if (Constants.dataLogging) {
      gyroReadLog.append(mDriveTrain.latestSwervePose.getRotation().getRadians());
      trajectoryHeadingReadLog.append((heading));
      trajectoryTimeStamp.append(currentTime);
      omegaRotation.append(adjustSpeeds.omegaRadiansPerSecond);
    }


    // Get the Module States
    SwerveModuleState[] moduleStates = mDriveTrain.swerveDriveKinematics.toSwerveModuleStates(adjustSpeeds);

    // Set the Module States
    mDriveTrain.frontLeftModule.setState(moduleStates[0]);
    mDriveTrain.frontRightModule.setState(moduleStates[1]);
    mDriveTrain.rearLeftModule.setState(moduleStates[2]);
    mDriveTrain.rearRightModule.setState(moduleStates[3]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsedTimer.get() >= mTrajectory.getTotalTimeSeconds();
  }
}