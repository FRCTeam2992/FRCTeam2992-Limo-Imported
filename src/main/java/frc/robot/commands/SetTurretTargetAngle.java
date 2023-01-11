
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;

public class SetTurretTargetAngle extends CommandBase {

    private Turret mTurret;

    private boolean mSetTarget;
    private double mDefaultTarget;

    public SetTurretTargetAngle(Turret subsystem, boolean setTarget, double defaultTarget) {

        mTurret = subsystem;
        mSetTarget = setTarget;
        mDefaultTarget = defaultTarget;

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        if (mSetTarget) {
            mTurret.turretTarget = mDefaultTarget;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }
}