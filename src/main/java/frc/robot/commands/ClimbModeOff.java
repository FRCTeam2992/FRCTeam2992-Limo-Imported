// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ClimbModeOff extends CommandBase {

  private Climb mClimb;
  private Intake mIntake;
  private Drivetrain mDrivetrain;

  public ClimbModeOff(Climb subsystem, Intake intakeSubsystem, Drivetrain driveSubsystem) {
    mClimb = subsystem;
    mIntake = intakeSubsystem;
    // addRequirements(intakeSubsystem);
    mDrivetrain = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimb.setClimbMode(false);
    mDrivetrain.setInSlowMode(false);
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
