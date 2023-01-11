// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoFunnel;

public class SpinCargoFunnel extends CommandBase {

  private CargoFunnel mCargoFunnel;
  private double mFunnelSpeed;

  public SpinCargoFunnel(CargoFunnel subsystem, double funnelSpeed) {

    mCargoFunnel = subsystem;
    mFunnelSpeed = funnelSpeed;

    addRequirements(mCargoFunnel);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mCargoFunnel.setFunnelSpeed(mFunnelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
