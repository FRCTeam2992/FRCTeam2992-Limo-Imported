// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private WPI_TalonSRX hoodMotor;

  private CANCoder hoodEncoder;

  private double hoodPosition = Constants.defaultHoodPosition;
  private boolean isAiming = false;       // Are we doing auto aiming right now
  private boolean targetAcquired = false;

  public PIDController hoodPID;

  public MedianFilter hoodMedianFilter;

  int dashboardCounter = 0;

  public ShooterHood() {
    hoodMotor = new WPI_TalonSRX(33);
    hoodMotor.setInverted(true);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    addChild("hoodMotor", hoodMotor);

    hoodEncoder = new CANCoder(33, "CanBus2");

    hoodEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    hoodEncoder.configSensorDirection(true);

    hoodPID = new PIDController(Constants.hoodP, Constants.hoodI, Constants.hoodD);
    hoodPID.setTolerance(Constants.hoodTolerance);
    hoodPID.disableContinuousInput();
    hoodPID.setIntegratorRange(-0.2, 0.2);

    hoodMotor.configRemoteFeedbackFilter(hoodEncoder, 0);

    hoodMedianFilter = new MedianFilter(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (++dashboardCounter >= 5) {
      SmartDashboard.putNumber("Hood Encoder Angle", getEncoderAngle());
      SmartDashboard.putNumber("Hood Target", hoodPosition);
      // SmartDashboard.putBoolean("Hood Aiming", isAiming());
      // SmartDashboard.putBoolean("Hood On Target", atTarget());
      dashboardCounter = 0;
    }
  }

  public void setHoodSpeed(double speed) {

    hoodMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setHoodTarget(double position) {
    position = Math.max(position, Constants.minHoodPosition);
    position = Math.min(position, Constants.maxHoodPosition);
    hoodPosition = position;                    // Save the last targeted angle
  }

  public double getHoodTarget() {
    return hoodPosition;
  }

  public void setToTarget() {
    double position = Math.max(hoodPosition, Constants.minHoodPosition);
    position = Math.min(position, Constants.maxHoodPosition);

    if(Math.abs(getEncoderAngle() - position) > 15.0){
      hoodPID.reset();
    }

    double power = hoodPID.calculate(getEncoderAngle(), position) + Constants.hoodF;
    power = Math.min(power, .5);
    power = Math.max(power, -.4);

    hoodMotor.set(ControlMode.PercentOutput, power);
  }

  public double getEncoderAngle() {
    double tempAngle = hoodEncoder.getAbsolutePosition() + Constants.hoodEncoderOffset;

    if (tempAngle < -180.0) {
      tempAngle += 360.0;
    } else if (tempAngle > 180) {
      tempAngle -= 360;
    }
    return hoodMedianFilter.calculate(tempAngle);
  }

  public double getHoodAngle() {
    double angle = getEncoderAngle() * Constants.hoodAngleRatio;

    return angle;
  }

  public boolean atTarget() {
    return ((Math.abs(getEncoderAngle() - hoodPosition) < Constants.hoodTolerance) &&
          !(isAiming() && !targetAcquired));
  }

  public boolean isAiming() {
    return isAiming;
  }

  public void setAiming(boolean isAiming) {
    this.isAiming = isAiming;
  }

  public boolean isTargetAcquired() {
    return targetAcquired;
  }

  public void setTargetAcquired(boolean targetAcquired) {
    this.targetAcquired = targetAcquired;
  }

  

}
