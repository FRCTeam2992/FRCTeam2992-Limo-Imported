// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopLift extends SubsystemBase {
  /** Creates a new TopLift. */
  private WPI_VictorSPX topLiftMotor;

  private boolean isCommanded = false;               // Is Top Lift commanded to run in default command mode
  private double commandedSpeed = 0.0;             // Speed to run if its commanded on during default command

  public TopLift() {
    topLiftMotor = new WPI_VictorSPX(23);
    topLiftMotor.setInverted(true);
    topLiftMotor.setNeutralMode(NeutralMode.Brake);   
    topLiftMotor.setStatusFramePeriod(1, 255);
    topLiftMotor.setStatusFramePeriod(2, 254);
    topLiftMotor.setStatusFramePeriod(3, 253);
    topLiftMotor.setStatusFramePeriod(4, 252);
    topLiftMotor.setStatusFramePeriod(8, 251);
    topLiftMotor.setStatusFramePeriod(10, 250);
    topLiftMotor.setStatusFramePeriod(12, 249);
    topLiftMotor.setStatusFramePeriod(13, 248);
    topLiftMotor.setStatusFramePeriod(14, 247);
    topLiftMotor.setStatusFramePeriod(21, 246);

    addChild("topLiftMotor", topLiftMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTopLiftSpeed(double speed) {
    topLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isCommanded() {
    return isCommanded;
  }

  public void setCommanded(boolean isCommanded) {
    this.isCommanded = isCommanded;
  }

  public double getCommandedSpeed() {
    return commandedSpeed;
  }

  public void setCommandedSpeed(double commandedSpeed) {
    this.commandedSpeed = commandedSpeed;
  }

  public void reset() {
    setCommanded(false);
    setCommandedSpeed(0.0);
  }
  
}
