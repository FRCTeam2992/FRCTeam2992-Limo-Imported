// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeCommanded extends CommandBase {

  private Intake mIntake;
  private boolean mCommanded;
  private double mSpeed;

  public SetIntakeCommanded(Intake subsystem, boolean commanded, double speed) {
    mIntake = subsystem;
    mCommanded = commanded;
    mSpeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntake.setIntakeCommanded(mCommanded);
    mIntake.setSpeedCommanded(mSpeed);
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
