/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ChangeMainShooterSpeed extends CommandBase {

  // Subsystem Instance
  private Shooter mShooter;

  // Saved Variables
  private int mChangeSpeed;

  public ChangeMainShooterSpeed(Shooter subsystem, int changeSpeed) {
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
    double changeSpeed = Math.floor((mShooter.getMainShooterTargetRPM() + mChangeSpeed) / mChangeSpeed) * mChangeSpeed ;

    changeSpeed = Math.max(0, changeSpeed);

    mShooter.setMainShooterTargetRPM(changeSpeed);
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
