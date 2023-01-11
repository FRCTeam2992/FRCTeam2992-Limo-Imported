// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.SpinBottomLift;
import frc.robot.commands.SpinCargoFunnel;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTopLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DejamBallPath extends SequentialCommandGroup {
  /** Creates a new DejamBallPath. */
  public DejamBallPath(Intake mIntake, CargoFunnel mCargoFunnel, BottomLift mBottomLift, IntakeDeploy mIntakeDeploy) {
    addCommands(
      //new DeployIntake(mIntake, true),
      new ParallelCommandGroup(
        new DeployIntake(mIntakeDeploy),
        new SpinBottomLift(mBottomLift, -1),
        new SpinCargoFunnel(mCargoFunnel, -.75),
        new SpinIntake(mIntake, -.75)
      )
    );
  }
}
