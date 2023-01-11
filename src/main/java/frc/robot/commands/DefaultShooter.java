// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class DefaultShooter extends CommandBase {
  /** Creates a new StopIntake. */
  private Shooter mShooter;
  private Turret mTurret;
  private CargoBallInterpolator mInterpolator;
  
  public DefaultShooter(Shooter shooter, Turret turret, CargoBallInterpolator interpolator) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mTurret = turret;
    mInterpolator = interpolator;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooter.isShooterCommanded()) {
      // Shooterwas last commanded on so spin it
      // CommandScheduler.getInstance().schedule(new StartShooter(mShooter));
      if (!mTurret.limeLightCamera.hasTarget() && Robot.mRobotContainer.controller1.getLeftBumper()) {
          // Pose2d robotPose = Robot.mRobotContainer.mDrivetrain.swerveDrivePoseEstimator.getEstimatedPosition();
          // Transform2d toTarget = robotPose.minus(Constants.goalPose);
          // double distance = (100 / 2.54) * toTarget.getTranslation().getDistance(new Translation2d());
          // SmartDashboard.putNumber("PoseEst distance", distance);
          // double mainSpeed = mInterpolator.calcMainShooterSpeed(distance);
          // double secondarySpeed = mInterpolator.calcSecondShooterSpeed(distance);
          // mShooter.setMainShooterTargetRPM(mainSpeed);
          // mShooter.setSecondaryShooterTargetRPM(secondarySpeed);        
      }
      mShooter.setMainShooterToTargetRPM();
      mShooter.setSecondaryShooterToTargetRPM();
    } else {
      //CommandScheduler.getInstance().schedule(new StopShooter(mShooter));
      mShooter.setMainShooterPower(0.0);
      mShooter.setSecondaryShooterPower(0.0);
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
