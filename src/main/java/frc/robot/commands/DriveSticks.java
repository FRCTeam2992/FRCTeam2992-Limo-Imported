// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveSticks extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;

  // Joystick Settings
  private boolean isLeftStrafe = true;

  // Command States
  private double gyroTarget;
  private boolean gyroTargetRecorded;

  public DriveSticks(Drivetrain subsystem) {
    // Subsystem Instance
    mDriveTrain = subsystem;

    // Set the Subsystem Requirement
    addRequirements(mDriveTrain);
  }

  // Called when the command is initially scheduled./
  @Override
  public void initialize() {
    // Reset the Target Recorded State
    gyroTargetRecorded = false;

    // Get the Joystick Settings
    isLeftStrafe = SmartDashboard.getBoolean("isLeftStrafe", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Joystick Inputs (x1 = Strafe, y1 = Speed, x2 = Rotation)
    double x1;
    double y1;
    double x2;

    if (isLeftStrafe) {
      x1 = -Robot.mRobotContainer.getController0().getLeftX();
      y1 = -Robot.mRobotContainer.getController0().getLeftY();
      x2 = -Robot.mRobotContainer.getController0().getRightX();
    } else {
      x1 = -Robot.mRobotContainer.getController0().getRightX();
      y1 = -Robot.mRobotContainer.getController0().getRightY();
      x2 = -Robot.mRobotContainer.getController0().getLeftX();
    }

    // Get the Joystick Magnitude
    double xyMagnitude = Math.sqrt((x1 * x1) + (y1 * y1));

    // Check the Magnitude Deadband
    if (Math.abs(xyMagnitude) <= Constants.joystickDeadband) {
      x1 = 0.0;
      y1 = 0.0;
    } else {
      // Get the Polar Angle
      double xyAngle = Math.atan2(y1, x1);

      // Smooth the X and Y Axis
      double smoothedXYMagnitude = (Constants.joystickXYSmoothFactor * Math.pow(xyMagnitude, 3.0))
          + ((1.0 - Constants.joystickXYSmoothFactor) * xyMagnitude);

      // Normalize the Magnitude
      smoothedXYMagnitude = Math.max(-1.0, Math.min(1.0, smoothedXYMagnitude));

      // Convert from Polar to X and Y Coordinates
      x1 = smoothedXYMagnitude * Math.cos(xyAngle);
      y1 = smoothedXYMagnitude * Math.sin(xyAngle);
    }

    // Check the Rotation Deadband
    if (Math.abs(x2) <= Constants.joystickDeadband) {
      x2 = 0.0;
    } else {
      // Smooth the Rotation Axis
      // x2 = (Constants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
      // + ((1.0 - Constants.joystickRotationSmoothFactor) * x2);

      // Smooth the Rotation Axis and Apply Inverse Deadband
      double tempInverseDeadband = Constants.joystickRotationInverseDeadband;

      if (mDriveTrain.isInSlowMode()) {
        tempInverseDeadband /= 0.3;
      }

      if (x2 >= 0.0) {
        x2 = tempInverseDeadband
            + (1.0 - tempInverseDeadband) * ((Constants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
                + ((1.0 - Constants.joystickRotationSmoothFactor) * x2));
      } else {
        x2 = -tempInverseDeadband
            + (1.0 - tempInverseDeadband) * ((Constants.joystickRotationSmoothFactor * Math.pow(x2, 3.0))
                + ((1.0 - Constants.joystickRotationSmoothFactor) * x2));
      }
    }

    // // Check if LimeLight Button Pressed
    // if (Robot.m_robotContainer.autoAimButton.get()) {
    // // Turn On the LimeLight
    // mDriveTrain.limeLightCamera.setLedMode(LedMode.On);

    // // Check if LimeLight Has a Target
    // if (mDriveTrain.limeLightCamera.hasTarget() &&
    // Math.abs(mDriveTrain.limeLightCamera.getTargetXOffset()) > .5) {
    // // Calculate the Drive Aim Correction
    // x2 = -mDriveTrain.limeLightCamera.getTargetXOffset() * Constants.driveAimP;

    // gyroTargetRecorded = false;
    // }
    // } else {
    // // Turn Off the LimeLight
    // mDriveTrain.limeLightCamera.setLedMode(LedMode.Off);
    // }

    // Check for Movement
    if (Math.abs(x1) > 0.0 || Math.abs(y1) > 0.0 || Math.abs(x2) > 0.0) {

      // Demo Slow Mode
      // x1 /= 4;
      // y1 /= 4;
      // x2 /= 4;

      // Slow the Rotation
      x2 *= (2.0 / 3.0);

      // Check for Slow Mode
      if (mDriveTrain.isInSlowMode()) {
        x1 *= 0.6;
        y1 *= 0.6;
        x2 *= 0.3;
      }

      // Gyro Input (-180 to 180)
      double gyroValue = mDriveTrain.getGyroYaw();

      if (Math.abs(x1) <= Constants.joystickDeadband && Math.abs(y1) <= Constants.joystickDeadband) {
        gyroTargetRecorded = false;
      }

      // Gyro Correction
      if (Math.abs(x2) <= Constants.joystickDeadband && Constants.isGyroCorrected) {

        // Check for Recorded Value
        if (gyroTargetRecorded) {
          // Get the Gyro Value
          double tempGyroValue = gyroValue;

          // Normalize the Target Angle (-180 - 180)
          if (gyroTarget < -180.0) {
            gyroTarget += 360.0;
          } else if (gyroTarget > 180) {
            gyroTarget -= 360.0;
          }

          // Normalize the Gyro Angle (-180 - 180)
          if (tempGyroValue > 180.0) {
            tempGyroValue -= 360;
          } else if (tempGyroValue < -180) {
            tempGyroValue += 360;
          }

          // Get the Gyro Error
          double gyroError = gyroTarget + tempGyroValue;

          // Normalize the Gyro Error (-180 - 180)
          if (gyroError > 180.0) {
            gyroError -= 360.0;
          } else if (gyroError < -180.0) {
            gyroError += 360.0;
          }

          // Calculate Correction Speed
          x2 = gyroError * Constants.driveGyroP;
        } else {
          // Record a Gyro Value
          gyroTarget = -gyroValue;
          gyroTargetRecorded = true;
        }
      } else {
        // Reset the Target Recorded State
        gyroTargetRecorded = false;
      }

      // Calculate the Swerve States
      double[] swerveStates;

      // Check for Field Centric Enabled
      if (Constants.isFieldCentric && !Robot.mRobotContainer.controller0.getLeftBumper()) {
        swerveStates = mDriveTrain.swerveController.calculate(x1, y1, x2, gyroValue);
        // SmartDashboard.putBoolean("Is Field Oriented", true);
      } else {
        swerveStates = mDriveTrain.swerveController.calculate(x1, y1, x2);
        // SmartDashboard.putBoolean("Is Field Oriented", false);

      }

      // Get the Swerve Modules
      SwerveModuleFalconFalcon frontLeft = mDriveTrain.frontLeftModule;
      SwerveModuleFalconFalcon frontRight = mDriveTrain.frontRightModule;
      SwerveModuleFalconFalcon rearLeft = mDriveTrain.rearLeftModule;
      SwerveModuleFalconFalcon rearRight = mDriveTrain.rearRightModule;

      // Command the Swerve Modules
      if (Constants.isVelocityControlled) {
        frontLeft.setDriveVelocity(swerveStates[0], swerveStates[1]);
        frontRight.setDriveVelocity(swerveStates[2], swerveStates[3]);
        rearLeft.setDriveVelocity(swerveStates[4], swerveStates[5]);
        rearRight.setDriveVelocity(swerveStates[6], swerveStates[7]);
      } else {
        frontLeft.setDrive(swerveStates[0], swerveStates[1]);
        frontRight.setDrive(swerveStates[2], swerveStates[3]);
        rearLeft.setDrive(swerveStates[4], swerveStates[5]);
        rearRight.setDrive(swerveStates[6], swerveStates[7]);
      }
    } else {
      mDriveTrain.stopDrive();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stopDrive();
  }
}
