// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {
  /** Creates a new IntakeDeploy. */
  private CANSparkMax intakeDeployMotor;

  private DigitalInput intakeLimitSwitch;

  private boolean intakeDeployedState = false;

  private PIDController intakePIDContorller;

  private int dashboardCounter = 0;

  public IntakeDeploy() {
    intakeDeployMotor = new CANSparkMax(25, MotorType.kBrushless);
    intakeDeployMotor.setIdleMode(IdleMode.kBrake);
    intakeDeployMotor.setInverted(true);

    intakeLimitSwitch = new DigitalInput(2);

    intakePIDContorller = new PIDController(Constants.intakeP, Constants.intakeI, Constants.intakeD);
    intakePIDContorller.setTolerance(Constants.hoodTolerance);
    intakePIDContorller.disableContinuousInput();
    intakePIDContorller.setIntegratorRange(-0.2, 0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++dashboardCounter >= 5) {
      // SmartDashboard.putNumber("Intake Deploy Motor Encoder", getEncoderAngle());
    //   // SmartDashboard.putBoolean("Intake Deploy Limit Switch", getLimitSwitch());
    //   // SmartDashboard.putBoolean("Intake State", getIntakeDeployedState());

    //   dashboardCounter = 0;
    }

    if (!getLimitSwitch() && !intakeDeployedState && Math.abs(getEncoderAngle()) > 0.2) {
      initIntakeDeployMotor(0.0);
    }

  }

  public boolean getLimitSwitch() {
    return intakeLimitSwitch.get();
  }

  public double getEncoderAngle() {
    return intakeDeployMotor.getEncoder().getPosition();
  }

  public void deployIntake() {

    double power = intakePIDContorller.calculate(getEncoderAngle(), Constants.maxIntakeEncoderAngle);
    power = MathUtil.clamp(power, -.2, 1);
    if ((Math.abs(getEncoderAngle() - Constants.maxIntakeEncoderAngle) < Constants.intakeTolerance)) {
      power = 0;
    }

    intakeDeployMotor.set(power);
    // SmartDashboard.putNumber("deploy power", power);
  }

  public void retractIntake() {
    double power = intakePIDContorller.calculate(getEncoderAngle(), Constants.minIntakeEncoderAngle);
    power = MathUtil.clamp(power, -.5, .2);
    if ((Math.abs(getEncoderAngle() - Constants.minIntakeEncoderAngle) < Constants.intakeTolerance)) {
      power = Constants.intakeF;
    }
    intakeDeployMotor.set(power);
    // SmartDashboard.putNumber("retract power", power);
  
  }

  
  public boolean getIntakeDeployedState() {
    return intakeDeployedState;
  }

  public void setIntakeDeployedState(boolean intakeDeployedState) {
    this.intakeDeployedState = intakeDeployedState;
  }

  public void initIntakeDeployMotor(double position) {
    intakeDeployMotor.getEncoder().setPosition(position);
  }

  public void setIntakeDeployedSpeed(double speed){
    intakeDeployMotor.set(speed);
  }
}
