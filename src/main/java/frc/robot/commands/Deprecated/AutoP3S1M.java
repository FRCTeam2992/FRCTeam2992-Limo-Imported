// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Deprecated;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.AutoLimelightHood;
import frc.robot.commands.AutoLimelightMainShooter;
import frc.robot.commands.AutoLimelightSecondShooter;
import frc.robot.commands.AutoTurretAimAutonomous;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.paths.StraightPath;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TopLift;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoP3S1M extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public AutoP3S1M(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret, 
      CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel, TopLift mTopLift,
      BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new NewHoodTarget(mShooterHood, 0.0),
      //new StartHood(mShooterHood),
      //new SetShooterSpeedTargets(mShooter, 1500, 2000),
      new SetShooterCommanded(mShooter, true),
      new AutoTurretAimAutonomous(mTurret, false, 0),
      new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
      new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
      new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
      new SequentialCommandGroup(
        // new WaitCommand(0.040),
        new AutoShoot(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(1.0),
        new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, true),
        new AutoFollowPath(mDrivetrain, new StraightPath(2.0, 0).generateSwerveTrajectory(), false, false, 0)
      )
    );
  }
}
