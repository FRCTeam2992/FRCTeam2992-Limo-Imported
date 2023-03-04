// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ChangeSecondaryShooterSpeed extends CommandBase {

  // Subsystem Instance
  private Shooter mShooter;

  // Saved Variables
  private int mChangeSpeed;

  public ChangeSecondaryShooterSpeed(Shooter subsystem, int changeSpeed) {
    // Subsystem Instance
    mShooter = subsystem;

    // Saved Variables
    mChangeSpeed = changeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double changeSpeed = Math.floor((mShooter.getSecondaryShooterTargetRPM() +
    // mChangeSpeed) / mChangeSpeed)
    // * mChangeSpeed;

    // changeSpeed = Math.max(0, changeSpeed);

    // mShooter.setSecondaryShooterTargetRPM(changeSpeed);
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
