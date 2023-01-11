// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;

  public ResetOdometry(Drivetrain subsystem) {
    // Subsystem Instance
    mDriveTrain = subsystem;

    // Set the Subsystem Requirement
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset the Gyro
    mDriveTrain.navx.zeroYaw();
    mDriveTrain.gyroOffset = 0.0;
    Pose2d pose = mDriveTrain.latestSwervePoseEstimate;
    mDriveTrain.setPoseEstimatePosition(true, new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    mDriveTrain.setOdometryPosition(true, new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

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
