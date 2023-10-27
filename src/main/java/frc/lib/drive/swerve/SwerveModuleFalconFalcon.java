package frc.lib.drive.swerve;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleFalconFalcon {

    // Saved Variables
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder encoderInput;
    private double encoderOffset;
    private double wheelDiameter;
    private double wheelGearRatio;
    private double maxDriveSpeed;
    private PIDController turnPID;

    private DutyCycleOut percentOutputControlRequest;
    private VelocityDutyCycle velocityControlRequest;

    public SwerveModuleFalconFalcon(TalonFX driveMotor, TalonFX turnMotor, CANcoder encoderInput,
            double encoderOffset, PIDController turnPID, double wheelDiameter, double wheelGearRatio,
            double maxDriveSpeed) {
        // Saved Variables
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoderInput = encoderInput;
        this.encoderOffset = encoderOffset;
        this.turnPID = turnPID;
        this.wheelDiameter = wheelDiameter;
        this.wheelGearRatio = wheelGearRatio;
        this.maxDriveSpeed = maxDriveSpeed;

        percentOutputControlRequest = new DutyCycleOut(0.0);
        velocityControlRequest = new VelocityDutyCycle(0.0);
    }

    public void setDriveSpeed(double speed) {
        driveMotor.setControl(percentOutputControlRequest.withOutput(speed));
    }

    public void setTurnSpeed(double speed) {
        turnMotor.setControl(percentOutputControlRequest.withOutput(speed));
    }

    public void stop() {
        setDriveSpeed(0.0);
        setTurnSpeed(0.0);
    }

    public void setTurnAngle(double degrees) {
        degrees = Math.min(Math.max(degrees, -180.0), 180.0);
        setTurnSpeed(turnPID.calculate(getEncoderAngle(), degrees));
    }

    public void setVelocityMeters(double speed) {
        double RPM = (speed * wheelGearRatio * 60) / (wheelDiameter * Math.PI);

        driveMotor.setControl(velocityControlRequest.withVelocity(RPM / 60.0));
    }

    public void setDrive(double speed, double angle) {
        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setDriveSpeed(speed);
        setTurnAngle(angle);
    }

    public void setDriveVelocity(double speedPercent, double angle) {
        double speed = speedPercent * maxDriveSpeed;

        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }

    public boolean atAngle() {
        return turnPID.atSetpoint();
    }

    public double getEncoderAngle() {
        double tempAngle = encoderInput.getAbsolutePosition().getValue() * 360.0 - encoderOffset;

        // Not sure if -180 adjust needed.  Not sure why this was here before
        //tempAngle += 180.0;

        if (tempAngle < -180.0) {
            tempAngle += 360.0;
        } else if (tempAngle > 180.0) {
            tempAngle -= 360.0;
        }

        return -tempAngle;
    }

    public double getWheelSpeedMeters() {
        double RPM = (driveMotor.getVelocity().getValue() * 60);

        double speed = (RPM * wheelDiameter * Math.PI) / (wheelGearRatio * 60);

        return speed;
    }

    public double getWheelPositionMeters() {
        double position = driveMotor.getSelectedSensorPosition();
        
        return position;
    }    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelSpeedMeters(), Rotation2d.fromDegrees(getEncoderAngle()));
    }

    

    public void setState(SwerveModuleState state) {
        double angle = state.angle.getDegrees();
        double speed = state.speedMetersPerSecond;

        //angle -= 180.0;


        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

            speed = -speed;
        }

        setVelocityMeters(speed);
        setTurnAngle(angle);
    }
//TODO finish this

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getWheelPositionMeters(), Rotation2d.fromDegrees(getEncoderAngle()));
    }
    public void setPosition(SwerveModulePosition position) {
        double angle = position.angle.getDegrees();
       // double distance = position.distanceMeters;

        //angle -= 180.0;


        if (Math.abs(getEncoderAngle() - angle) > 90.0) {
            if (angle > 0) {
                angle -= 180.0;
            } else {
                angle += 180.0;
            }

          //  speed = -speed;
        }

        //setVelocityMeters(speed);
        setTurnAngle(angle);
    }

}
