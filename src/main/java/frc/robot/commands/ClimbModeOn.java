// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;

public class ClimbModeOn extends CommandBase {

  private Climb mClimb;
  private IntakeDeploy mIntakeDeploy;
  private Intake mIntake;
  private Drivetrain mDrivetrain;

  public ClimbModeOn(Climb climbSubsystem, IntakeDeploy intakeDeploySubsystem, Intake intakeSubsystem, 
        Drivetrain driveSubsystem) {
    mClimb = climbSubsystem;
    mIntakeDeploy = intakeDeploySubsystem;
    mIntake = intakeSubsystem;
    mDrivetrain = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.setInSlowMode(true);
    mClimb.setClimbMode(true);
    mIntakeDeploy.setIntakeDeployedState(false);
    mIntake.setIntakeCommanded(false);

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
