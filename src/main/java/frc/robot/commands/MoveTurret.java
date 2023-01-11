// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

    private double mTurretSpeed = 0;

    private Turret mTurret;

    public MoveTurret(Turret subsystem, double speed) {
        mTurret = subsystem;

        mTurretSpeed = speed;

        addRequirements(subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        mTurret.setTurretSpeed(mTurretSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.setTurretSpeed(0.0);
    }
}