// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomLift;

public class SpinBottomLiftSensor extends CommandBase {
  /** Creates a new SpinBottomLift. */
  private BottomLift mBottomLift;

  private double mBottomLiftSpeed;

  private Timer delayTimer;

  public SpinBottomLiftSensor(BottomLift subsystem, double bottomLiftSpeed) {
    mBottomLift = subsystem;
    mBottomLiftSpeed = bottomLiftSpeed;

    delayTimer = new Timer();

    addRequirements(mBottomLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mBottomLift.getBottomSensorState() || mBottomLift.getTopSensorState()) {
      delayTimer.start();
      if (delayTimer.get() >= .08) {
        mBottomLift.setBottomLiftSpeed(0.0);

      }

    } else {
      mBottomLift.setBottomLiftSpeed(mBottomLiftSpeed);
      delayTimer.reset();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mBottomLift.setBottomLiftSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
