// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private WPI_TalonFX leftClimbMotor;
  private WPI_TalonFX rightClimbMotor;

  private boolean climbMode = false;
  private double targetClimbPosition = 0.0;

  private int dashboardCounter = 0;

  public Climb() {
    leftClimbMotor = new WPI_TalonFX(40);
    leftClimbMotor.setInverted(true);
    leftClimbMotor.setNeutralMode(NeutralMode.Brake);
    addChild("leftClimbMotor", leftClimbMotor);

    rightClimbMotor = new WPI_TalonFX(41);
    rightClimbMotor.setInverted(true);
    rightClimbMotor.setNeutralMode(NeutralMode.Brake);
    addChild("rightClimbMotor", rightClimbMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++dashboardCounter >= 5) {
      SmartDashboard.putBoolean("Climb Mode", getClimbMode());
      // SmartDashboard.putNumber("Left Climb Position", getLeftEncoderAngle());
      // SmartDashboard.putNumber("Right Climb Position", getRighttEncoderAngle());
      dashboardCounter = 0;
    }
  }

  public void setClimbSpeed(double speed) {
    if (getClimbMode()) {
      leftClimbMotor.set(ControlMode.PercentOutput, speed);
      rightClimbMotor.set(ControlMode.PercentOutput, speed);
    }
  }

  public void setClimbSpeedSmart(double speed) {

    if (!climbMode) {
      // Not in climb mode, so just stop the climber
      leftClimbMotor.set(ControlMode.PercentOutput, 0);
      rightClimbMotor.set(ControlMode.PercentOutput, 0);
      return;
    }

    // All code from here down only runs if in climb mode
    // if (climbOverride){
    //     // Climb mode is on and override button pressed to let climber move no matter what
    //     teleClimbMotor.set(ControlMode.PercentOutput, speed);
    //     return;
    //   }

    // All code from here down only runs if in climb mode and not using override

    // Left Side
    if (getLeftEncoderAngle() < Constants.topClimbTarget && getLeftEncoderAngle() > Constants.bottomClimbTarget) {
      // We are within the climb range of 0 to topTeleClimbLimit
      if (getLeftEncoderAngle() > Constants.topClimbSlowZone && speed > 0) {
        // Nearing the top so slow down if moving up
        leftClimbMotor.set(ControlMode.PercentOutput, speed * Constants.climbSlowModifier);
      } else if (getLeftEncoderAngle() < Constants.bottomClimbSlowZone && speed < 0) {
        // Nearing the bottom so slow down if moving down
        leftClimbMotor.set(ControlMode.PercentOutput, speed * Constants.climbSlowModifier);
      } else {
        // We are not near the top or bottom so full speed is OK
        leftClimbMotor.set(ControlMode.PercentOutput, speed);
      }
    }
    else {
      // We are outside the climb range.  Only allow motion in proper direction
      if (getLeftEncoderAngle() <= 0 && speed > 0) {
        // We are below the "bottom" of climb range but moving up so its OK
        leftClimbMotor.set(ControlMode.PercentOutput, speed);
      }
      else if (getLeftEncoderAngle() >= Constants.topClimbTarget && speed < 0) {
        // We are above the "top" of climb range but moving down so its OK
        leftClimbMotor.set(ControlMode.PercentOutput, speed);
      }
      else {
        // Outside climb range and not moving towards climb range so stop
        leftClimbMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    // Right Side
    if (getRighttEncoderAngle() < Constants.topClimbTarget && getRighttEncoderAngle() > Constants.bottomClimbTarget) {
      // We are within the climb range of 0 to topTeleClimbLimit
      if (getRighttEncoderAngle() > Constants.topClimbSlowZone && speed > 0) {
        // Nearing the top so slow down if moving up
        rightClimbMotor.set(ControlMode.PercentOutput, speed * Constants.climbSlowModifier);
      } else if (getRighttEncoderAngle() < Constants.bottomClimbSlowZone && speed < 0) {
        // Nearing the bottom so slow down if moving down
        rightClimbMotor.set(ControlMode.PercentOutput, speed * Constants.climbSlowModifier);
      } else {
        // We are not near the top or bottom so full speed is OK
        rightClimbMotor.set(ControlMode.PercentOutput, speed);
      }
    }
    else {
      // We are outside the climb range.  Only allow motion in proper direction
      if (getRighttEncoderAngle() <= 0 && speed > 0) {
        // We are below the "bottom" of climb range but moving up so its OK
        rightClimbMotor.set(ControlMode.PercentOutput, speed);
      }
      else if (getRighttEncoderAngle() >= Constants.topClimbTarget && speed < 0) {
        // We are above the "top" of climb range but moving down so its OK
        rightClimbMotor.set(ControlMode.PercentOutput, speed);
      }
      else {
        // Outside climb range and not moving towards climb range so stop
        rightClimbMotor.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public boolean getClimbMode() {
    return climbMode;
  }

  public void setClimbMode(boolean climbMode) {
    this.climbMode = climbMode;
  }

  public double getLeftEncoderAngle() {
    return leftClimbMotor.getSelectedSensorPosition();
  }

  public double getRighttEncoderAngle() {
    return rightClimbMotor.getSelectedSensorPosition();
  }

  public void resetClimbMotors(){
    leftClimbMotor.setSelectedSensorPosition(0.0);
    rightClimbMotor.setSelectedSensorPosition(0.0);
    }

}
