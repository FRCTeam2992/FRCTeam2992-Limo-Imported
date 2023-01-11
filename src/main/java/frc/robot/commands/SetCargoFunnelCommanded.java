// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoFunnel;

public class SetCargoFunnelCommanded extends CommandBase {

  private CargoFunnel mCargoFunnel;
  private boolean mCommanded;
  private boolean mCheckSensor;
  private double mNoBallSpeed;
  private double mWithBallSpeed;
  private double mSensorDelay;

  public SetCargoFunnelCommanded (CargoFunnel subsystem, boolean commanded, boolean checkSensor,
            double noBallSpeed, double withBallSpeed, double sensorDelay) {
    mCargoFunnel = subsystem;
    mCommanded = commanded;
    mCheckSensor = checkSensor;
    mNoBallSpeed = noBallSpeed;
    mWithBallSpeed = withBallSpeed;
    mSensorDelay = sensorDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCargoFunnel.setCommanded(mCommanded);
    mCargoFunnel.setCheckSensor(mCheckSensor);
    mCargoFunnel.setNoBallSpeed(mNoBallSpeed);
    mCargoFunnel.setWithBallSpeed(mWithBallSpeed);
    mCargoFunnel.setSensorDelay(mSensorDelay);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
