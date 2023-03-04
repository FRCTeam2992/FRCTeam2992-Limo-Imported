// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoLimelightSecondShooter extends CommandBase {
  /** Creates a new AutoLimelightMainShooter. */
  private Turret mTurret;
  private Shooter mShooter;

  private CargoBallInterpolator mInterpolator;

  public AutoLimelightSecondShooter(Turret tSubsystem, Shooter sSubsytem, CargoBallInterpolator interpolator) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTurret = tSubsystem;
    mShooter = sSubsytem;

    mInterpolator = interpolator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTurret.limeLightCamera.hasTarget()) {

      double currentDistance = mTurret.limeLightCamera.getDistanceToTarget(Constants.cameraAngle,
          Constants.cameraHeight, Constants.goalHeight);

      double targetSpeed = mInterpolator.calcSecondShooterSpeed(currentDistance);

      // SmartDashboard.putNumber("Second Target Speed", targetSpeed);
      // mShooter.setSecondaryShooterTargetRPM(targetSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
