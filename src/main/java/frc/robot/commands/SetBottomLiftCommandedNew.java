// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomLift;

public class SetBottomLiftCommandedNew extends CommandBase {

  private BottomLift mBottomLift;
  private boolean mCommanded;
  private double mNoBallSpeed;
  private double mWithBallSpeed;
  private boolean mCheckSensor;

  public SetBottomLiftCommandedNew (BottomLift subsystem, boolean commanded,
            boolean checkSensor, double noBallSpeed, double withBallSpeed) {
    mBottomLift = subsystem;
    mCommanded = commanded;
    mCheckSensor = checkSensor;
    mNoBallSpeed = noBallSpeed;
    mWithBallSpeed = withBallSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBottomLift.setCommanded(mCommanded);
    mBottomLift.setNoBallSpeed(mNoBallSpeed);
    mBottomLift.setWithBallSpeed(mWithBallSpeed);
    mBottomLift.setCheckSensor(mCheckSensor);
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
