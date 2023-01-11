// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetSlowMode extends CommandBase {
  /** Creates a new SetSlowMode. */
  private Drivetrain mDrivetrain;

  private boolean inSlowMode = false;

  public SetSlowMode(Drivetrain subsystem, boolean newSlowMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = subsystem;
    inSlowMode = newSlowMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.setInSlowMode(inSlowMode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
