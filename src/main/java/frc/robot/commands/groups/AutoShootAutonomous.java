// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveTrainStopped;
import frc.robot.commands.HoodAtAngle;
import frc.robot.commands.SetBottomLiftCommandedNew;
import frc.robot.commands.SetCargoFunnelCommanded;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.SetTopLiftCommanded;
import frc.robot.commands.ShooterAtSpeed;
import frc.robot.commands.TurretOnTarget;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TopLift;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAutonomous extends SequentialCommandGroup {

  /** Creates a new AutoShoot. */
  public AutoShootAutonomous(CargoFunnel mCargoFunnel, BottomLift mBottomLift,
        Shooter mShooter, ShooterHood mShooterHood, Turret mTurret, Drivetrain mDrivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  addCommands(
    new ParallelCommandGroup(                       // Preshoot checks must be completed first
      new SetShooterCommanded(mShooter, true),      // Make sure shooter is running
      new ShooterAtSpeed(mShooter).withTimeout(2),
      new HoodAtAngle(mShooterHood).withTimeout(0.5),
      new TurretOnTarget(mTurret).withTimeout(0.5),
      new DriveTrainStopped(mDrivetrain).withTimeout(0.5),
      new WaitCommand(0.5)
      ),      
    new ParallelCommandGroup(                       // OK TO shoot
      new SetCargoFunnelCommanded(mCargoFunnel, true, false, 0.55, 0.55, 0.0),
      new SetBottomLiftCommandedNew(mBottomLift, true, false, 0.5, 0.5),
      new WaitCommand(5.0)
    )
    );
  }
}
