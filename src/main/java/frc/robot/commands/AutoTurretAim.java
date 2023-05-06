
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.vision.LimeLight.LedMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Turret;

public class AutoTurretAim extends CommandBase {

    private Turret mTurret;
    private Climb mClimb;

    private double turretSetAngle = 0.0;

    public AutoTurretAim(Turret subsystem, Climb climb) {
        addRequirements(subsystem);

        mTurret = subsystem;
        mClimb = climb;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        mTurret.limeLightCamera.resetMedianFilters();
        mTurret.setAutoAiming(true);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        mTurret.limeLightCamera.setLedMode(LedMode.On);

        if (mTurret.limeLightCamera.hasTarget()) {
            double xOffset = mTurret.limeLightCamera.getTargetXOffset();

            if (Math.abs(xOffset) > 0.5) {
                turretSetAngle = mTurret.getTurretAngle() + xOffset;
            }
            mTurret.goToAngle(turretSetAngle);
        }

        else {
            // Repleace stop turret with turret sticks logic if doesn't have target
            // mTurret.setTurretSpeed(0);
            double x = -Robot.mRobotContainer.controller1.getLeftX();
            double y = -Robot.mRobotContainer.controller1.getLeftY();
            double targetAngle;
            double xyMagnitude = Math.sqrt((x * x) + (y * y));

            // if (xyMagnitude >= Constants.turretJoystickDeadband &&
            // !mClimb.getClimbMode()){
            // if(xyMagnitude > 1){
            // x /= xyMagnitude;
            // y /= xyMagnitude;
            // }
            // if(Constants.isFieldCentric){
            // targetAngle = Turret.angleOverlap((Math.toDegrees(Math.atan2(y, x)) - 90) -
            // mTurret.getGyroYaw());
            // }
            // mTurret.goToAngle(Turret.angleOverlap(targetAngle));
            // // SmartDashboard.putNumber("TurretStick output", targetAngle);
            // } else if (Robot.mRobotContainer.controller1.getLeftBumper()) {
            // mTurret.stopTurret();
            // // Pose2d robotPose =
            // Robot.mRobotContainer.mDrivetrain.swerveDrivePoseEstimator.getEstimatedPosition();
            // // Transform2d toTarget = robotPose.minus(Constants.goalPose);
            // // double toTargetX = toTarget.getTranslation().getX();
            // // double toTargetY = toTarget.getTranslation().getY();
            // // double toTargetAngle = Turret.angleOverlap(180 -
            // Math.toDegrees(Math.atan2(toTargetY, toTargetX)));
            // // SmartDashboard.putNumber("toTargetAngle", toTargetAngle);
            // // SmartDashboard.putNumber("toTargetX", toTarget.getX() * 2.54 / 100);
            // // SmartDashboard.putNumber("toTargetY", toTarget.getY() * 2.54 / 100);
            // // mTurret.goToAngle(toTargetAngle - mTurret.getGyroYaw());
            // } else {
            // mTurret.stopTurret();
            // }

        }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.stopTurret();
        mTurret.setAutoAiming(false);
        mTurret.limeLightCamera.setLedMode(LedMode.Off);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}