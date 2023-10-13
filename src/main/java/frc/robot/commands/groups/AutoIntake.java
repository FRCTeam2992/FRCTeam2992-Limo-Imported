// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ChangeIntakeState;
import frc.robot.commands.SetBottomLiftCommandedNew;
import frc.robot.commands.SetCargoFunnelCommanded;
import frc.robot.commands.SetIntakeCommanded;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public AutoIntake(Intake mIntake, CargoFunnel mCargoFunnel, BottomLift mBottomLift, IntakeDeploy mIntakeDeploy, boolean isDeployed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChangeIntakeState(mIntakeDeploy, true),
      new SetIntakeCommanded(mIntake, true, 0.4),
      new SetCargoFunnelCommanded(mCargoFunnel, true, true, .5, .25, 0.0),
      // new SetBottomLiftCommanded(mBottomLift, true, true, 0.5, 0.0, 0.12),
      new SetBottomLiftCommandedNew(mBottomLift, true, true, 0.7,  0.4)
      );
  }
}
