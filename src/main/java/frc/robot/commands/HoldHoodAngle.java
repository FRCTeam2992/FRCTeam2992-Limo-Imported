// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class HoldHoodAngle extends CommandBase {
  /** Creates a new HoldHoodAngle. */
  private ShooterHood mShooterHood;
  private Turret mTurret;
  private CargoBallInterpolator mInterpolator;

  private double startHoodAngle;

  public HoldHoodAngle(ShooterHood subsystem, Turret turret, CargoBallInterpolator interpolator) {
    mShooterHood = subsystem;
    mTurret = turret;
    mInterpolator = interpolator;

    addRequirements(mShooterHood);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startHoodAngle = mShooterHood.getEncoderAngle();
    mShooterHood.setHoodTarget(startHoodAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mTurret.limeLightCamera.hasTarget() && Robot.mRobotContainer.controller1.getLeftBumper()) {
      // Pose2d robotPose = Robot.mRobotContainer.mDrivetrain.swerveDrivePoseEstimator.getEstimatedPosition();
      // Transform2d toTarget = robotPose.minus(Constants.goalPose);
      // double distance = (100 / 2.54) * toTarget.getTranslation().getDistance(new Translation2d());
      // double angle = mInterpolator.calcHoodPosition(distance);
      // mShooterHood.setHoodTarget(angle);  
    }
    
    mShooterHood.setToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooterHood.setHoodSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
