// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Turret;

public class TurretSticks extends CommandBase {

  private Turret mTurret;

  private Climb mClimb;

  /** Creates a new TurretSticks. */
  public TurretSticks(Turret turretSubsystem, Climb climbSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTurret = turretSubsystem;
    mClimb = climbSubsystem;

    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -Robot.mRobotContainer.controller1.getRightX();
    double y = -Robot.mRobotContainer.controller1.getRightY();
    double targetAngle;
    double xyMagnitude = Math.sqrt((x * x) + (y * y));

    // Compensate to lead for robot rotation -- try 3 samples worth of drive train rotation
    double turned = Robot.mRobotContainer.mDrivetrain.angleTurned;
    double cycles = Math.max(Math.min(2.0 * Math.abs(turned * turned * turned), 15.0), 5.0);

    if (xyMagnitude >= Constants.turretJoystickDeadband && !mClimb.getClimbMode()){
      if(xyMagnitude > 1){
        x /= xyMagnitude;
        y /= xyMagnitude;
      }
      if(Constants.isFieldCentric){
        targetAngle = Turret.angleOverlap((Math.toDegrees(Math.atan2(y, x)) - 90) - mTurret.getGyroYaw());
      } else {
        targetAngle = Turret.angleOverlap((Math.toDegrees(Math.atan2(y, x)) - 90));
      }

      // // Compensate to lead for robot rotation -- try 3 samples worth of drive train rotation
      // if (Math.abs(turned) > 0.5) {
      //   targetAngle -= cycles *turned;
      // }
      // SmartDashboard.putNumber("Angle Turned", Robot.mRobotContainer.mDrivetrain.angleTurned);
      if (Math.abs(targetAngle - mTurret.getTurretAngle()) > 1.0) {
         mTurret.goToAngle(Turret.angleOverlap(targetAngle));
      }
      // SmartDashboard.putNumber("TurretStick output", targetAngle);
  } else if (!mClimb.getClimbMode() && Robot.mRobotContainer.controller1.leftBumper().getAsBoolean()) {
      mTurret.stopTurret();
      // Pose2d robotPose = Robot.mRobotContainer.mDrivetrain.swerveDrivePoseEstimator.getEstimatedPosition();
      // Transform2d toTarget = robotPose.minus(Constants.goalPose);
      // double toTargetX = toTarget.getTranslation().getX();
      // double toTargetY = toTarget.getTranslation().getY();
      // double toTargetAngle = Turret.angleOverlap(180 - Math.toDegrees(Math.atan2(toTargetY, toTargetX)));
      // // SmartDashboard.putNumber("toTargetAngle", toTargetAngle);
      // // SmartDashboard.putNumber("toTargetX", toTarget.getX() * 2.54 / 100);
      // // SmartDashboard.putNumber("toTargetY", toTarget.getY() * 2.54 / 100);
      // targetAngle = toTargetAngle;
      // if (Math.abs(turned) > 0.5) {
      //   targetAngle -= cycles * turned;
      // }
      // if (Math.abs((targetAngle - mTurret.getGyroYaw()) - mTurret.getTurretAngle()) > 1.0 ) {
      //     mTurret.goToAngle(targetAngle - mTurret.getGyroYaw());
      // }
    } else {
      mTurret.stopTurret();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTurret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
