// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

public class AutoLimelightHood extends CommandBase {
  /** Creates a new AutoLimelightMainShooter. */
  private Turret mTurret;
  private ShooterHood mShooterHood;

  private CargoBallInterpolator mInterpolator;

  public AutoLimelightHood(Turret tSubsystem, ShooterHood sHSubsytem, CargoBallInterpolator interpolator) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTurret = tSubsystem;
    mShooterHood = sHSubsytem;

    mInterpolator = interpolator;
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooterHood.setAiming(true);
    mShooterHood.setTargetAcquired(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTurret.limeLightCamera.hasTarget()) {
      mShooterHood.setTargetAcquired(true);
      double currentDistance = mTurret.limeLightCamera.getDistanceToTarget(Constants.cameraAngle,
          Constants.cameraHeight, Constants.goalHeight);

      double targetAngle = mInterpolator.calcHoodPosition(currentDistance);

      // SmartDashboard.putNumber("Hood Target Angle", targetAngle);
      mShooterHood.setHoodTarget(targetAngle);
    } else {
      mShooterHood.setTargetAcquired(false);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooterHood.setAiming(false);
    mShooterHood.setTargetAcquired(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
