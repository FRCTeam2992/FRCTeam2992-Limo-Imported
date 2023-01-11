// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SetSwerveAngleSafe extends CommandBase {
    private Drivetrain mDrivetrain;

    private double mFLAngle;
    private double mFRAngle;
    private double mRLAngle;
    private double mRRAngle;

    public SetSwerveAngleSafe(Drivetrain subsystem, double flAngle, double frAngle, double rlAngle, double rrAngle) {
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
        // Make sure we are not powering the drive wheels
        mDrivetrain.frontLeftModule.setDriveSpeed(0.0);
        mDrivetrain.frontRightModule.setDriveSpeed(0.0);
        mDrivetrain.rearLeftModule.setDriveSpeed(0.0);
        mDrivetrain.rearRightModule.setDriveSpeed(0.0);

        // Check if moving slow enough to lock the wheels
        if (((mDrivetrain.getDistanceTraveled() / Constants.robotPeriod) < Constants.maxSpeedToX) &&
                ((mDrivetrain.getAngleTurned() / Constants.robotPeriod) < Constants.maxTurnToX)) {
            // We are moving/turning slow enough so its OK to lock wheels
            mDrivetrain.frontLeftModule.setTurnAngle(mFLAngle);
            mDrivetrain.frontRightModule.setTurnAngle(mFRAngle);
            mDrivetrain.rearLeftModule.setTurnAngle(mRLAngle);
            mDrivetrain.rearRightModule.setTurnAngle(mRRAngle);
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
