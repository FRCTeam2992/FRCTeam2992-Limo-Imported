// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetGyro extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;
  private double  mGyroOffset;

  public ResetGyro(Drivetrain subsystem) {
    // Subsystem Instance
    mDriveTrain = subsystem;
    mGyroOffset = 0.0;

    // Set the Subsystem Requirement
  }

  public ResetGyro(Drivetrain subsystem, double gyroOffset) {
    mDriveTrain = subsystem;
    mGyroOffset = gyroOffset;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the Gyro
    mDriveTrain.navx.zeroYaw();
    mDriveTrain.gyroOffset = mGyroOffset;
    Pose2d pose = mDriveTrain.latestSwervePoseEstimate;
    mDriveTrain.setPoseEstimatePosition(true, new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0.0)));
    mDriveTrain.setOdometryPosition(true, new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0.0)));

    // Reset the Odometry
    // Kill reset of odometry.  We want odometry to stay unchanged until next robot init
    // mDriveTrain.resetOdometry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
