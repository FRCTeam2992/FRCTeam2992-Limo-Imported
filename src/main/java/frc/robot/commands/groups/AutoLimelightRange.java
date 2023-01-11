// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.AutoLimelightHood;
import frc.robot.commands.AutoLimelightMainShooter;
import frc.robot.commands.AutoLimelightSecondShooter;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.StartHood;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLimelightRange extends ParallelCommandGroup {
  /** Creates a new AutoLimelightRange. */
  public AutoLimelightRange(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret,
      Climb mClimb, CargoBallInterpolator mInterpolator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoTurretAim(mTurret, mClimb),
      new StartHood(mShooterHood),
      new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
      new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
      new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator)
    );
  }
}
