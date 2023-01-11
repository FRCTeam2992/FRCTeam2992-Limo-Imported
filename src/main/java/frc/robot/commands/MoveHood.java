// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterHood;

public class MoveHood extends CommandBase {
  private ShooterHood mShooterHood;

  private double mHoodSpeed;

  public MoveHood(ShooterHood subsystem, double hoodSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooterHood = subsystem;

    mHoodSpeed = hoodSpeed;

    addRequirements(mShooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tempHoodSpeed = mHoodSpeed;

    if (inDeadZone()) {
      tempHoodSpeed = 0.0;
      
    } else if (inMinPZone()) {
      double distance = Constants.minHoodPosition - mShooterHood.getEncoderAngle();
      tempHoodSpeed = (distance * Constants.hoodPValueBottom) + 0.02; 

    } else if (inMaxPZone()) {
      double distance = Constants.maxHoodPosition - mShooterHood.getEncoderAngle();
      tempHoodSpeed = distance * Constants.hoodPValueTop;
    } 

    mShooterHood.setHoodSpeed(tempHoodSpeed);

    // SmartDashboard.putBoolean("In Max P-Zone", inMaxPZone());
    // SmartDashboard.putBoolean("In Min P-Zone", inMinPZone());
    // SmartDashboard.putBoolean("In Dead-Zone", inDeadZone());
    // SmartDashboard.putNumber("Temp Hood Speed", tempHoodSpeed);
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

  private boolean inMinPZone() {
    if (mHoodSpeed < 0.0 && ((mShooterHood.getEncoderAngle() < Constants.minHoodPZone)
        && (mShooterHood.getEncoderAngle() > Constants.minHoodPosition))) {
      return true;
    } else {
      return false;
    }
  }

  private boolean inMaxPZone() {
    if (mHoodSpeed > 0.0 && ((mShooterHood.getEncoderAngle() > Constants.maxHoodPZone)
        && (mShooterHood.getEncoderAngle() < Constants.maxHoodPosition))) {
      return true;
    } else {
      return false;
    }
  }

  public boolean inDeadZone() {
    if (((mHoodSpeed > 0.0) && (mShooterHood.getEncoderAngle() > Constants.maxHoodPosition))
        || ((mHoodSpeed < 0) && (mShooterHood.getEncoderAngle() < Constants.minHoodPosition))) {
      return true;
    } else {
      return false;
    }
  }

}
