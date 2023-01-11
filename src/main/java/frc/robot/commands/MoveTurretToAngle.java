
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;

public class MoveTurretToAngle extends CommandBase {

    private double mAngle = 0;

      private Turret mTurret;
    private Timer endTimer;

    public MoveTurretToAngle(Turret subsystem, double angle) {
        mTurret = subsystem;
        mAngle = angle;
        endTimer = new Timer();
        addRequirements(subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        endTimer.reset();
        endTimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mTurret.goToAngle(mAngle);
        if (Math.abs(mTurret.getClosedLoopError()) > 100.0) {
            endTimer.reset();
        }
    
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return (endTimer.get() > 0.25);
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.stopTurret();
    }
}