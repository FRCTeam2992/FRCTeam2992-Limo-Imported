// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Deprecated;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetSwerveAngle extends CommandBase {
  private Drivetrain mDrivetrain;

  private double mFLAngle;
  private double mFRAngle;
  private double mRLAngle;
  private double mRRAngle;

  public SetSwerveAngle(Drivetrain subsystem, double flAngle, double frAngle, double rlAngle, double rrAngle) {
    mDrivetrain = subsystem;
    mFLAngle = flAngle;
    mFRAngle = frAngle;
    mRLAngle = rlAngle;
    mRRAngle = rrAngle;

    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.frontLeftModule.setTurnAngle(mFLAngle);
    mDrivetrain.frontLeftModule.setDriveSpeed(0.0);
    mDrivetrain.frontRightModule.setTurnAngle(mFRAngle);
    mDrivetrain.frontRightModule.setDriveSpeed(0.0);
    mDrivetrain.rearLeftModule.setTurnAngle(mRLAngle);
    mDrivetrain.rearLeftModule.setDriveSpeed(0.0);
    mDrivetrain.rearRightModule.setTurnAngle(mRRAngle);
    mDrivetrain.rearRightModule.setDriveSpeed(0.0);
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
