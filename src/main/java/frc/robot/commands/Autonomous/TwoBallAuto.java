// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.AutoLimelightHood;
import frc.robot.commands.AutoLimelightMainShooter;
import frc.robot.commands.AutoLimelightSecondShooter;
import frc.robot.commands.AutoTurretAimAutonomous;
import frc.robot.commands.ChangeIntakeState;
import frc.robot.commands.NewHoodTarget;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.SetShooterSpeedTargets;
import frc.robot.commands.SetTurretTargetAngle;
import frc.robot.commands.StartHood;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShootAutonomous;
import frc.robot.commands.groups.StopAutoIntake;
import frc.robot.paths.TwoBallPath;
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
public class TwoBallAuto extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public TwoBallAuto(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret, 
      CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel, 
      BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyro(mDrivetrain, 226.5),
      new NewHoodTarget(mShooterHood, -67.5),
      new StartHood(mShooterHood),
      new SetShooterSpeedTargets(mShooter, 1700, 2700),
      new SetShooterCommanded(mShooter, true),
      new AutoTurretAimAutonomous(mTurret, true, 167),
      new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
      new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
      new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
      new SequentialCommandGroup(
        // new WaitCommand(0.040),
        new WaitCommand(2.0),
        new AutoShootAutonomous(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(1.0),
        new WaitCommand(1.0),
        new ChangeIntakeState(mIntakeDeploy, true),
        new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, true),
        new NewHoodTarget(mShooterHood, 112),
        new SetTurretTargetAngle(mTurret, true, 169),
        new SetShooterSpeedTargets(mShooter, 2200, 2750),
        new AutoFollowPath(mDrivetrain, new TwoBallPath(mDrivetrain, 226.5).generateSwerveTrajectory(), true, false, 0.0).withTimeout(7),
        new WaitCommand(0.5),
        new AutoShootAutonomous(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(3),
        new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy),
        new SetShooterCommanded(mShooter, false)
      )
    );
  }
}



// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Autonomous;


// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoFollowPath;
// import frc.robot.commands.AutoLimelightHood;
// import frc.robot.commands.AutoLimelightMainShooter;
// import frc.robot.commands.AutoLimelightSecondShooter;
// import frc.robot.commands.AutoTurretAimAutonomous;
// import frc.robot.commands.SetShooterCommanded;
// import frc.robot.commands.groups.AutoIntake;
// import frc.robot.commands.groups.AutoShootAutonomous;
// import frc.robot.commands.groups.StopAutoIntake;
// import frc.robot.paths.TwoBallPath;
// import frc.robot.subsystems.BottomLift;
// import frc.robot.subsystems.CargoFunnel;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.IntakeDeploy;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterHood;
// import frc.robot.subsystems.TopLift;
// import frc.robot.subsystems.Turret;
// import frc.lib.Ranging.CargoBallInterpolator;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TwoBallAuto extends ParallelCommandGroup {
//   /** Creates a new TwoBallAuto. */
//   public TwoBallAuto(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret, 
//   CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel, TopLift mTopLift,
//   BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       new SetShooterCommanded(mShooter, true),
//       new AutoTurretAimAutonomous(mTurret, false, 0),
//       new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
//       new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
//       new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
//       new SequentialCommandGroup(
//         new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy).withTimeout(2),
//         new AutoFollowPath(mDrivetrain, new TwoBallPath(mDrivetrain, 226.5).generateSwerveTrajectory(), true, true, 226.5).withTimeout(5),
//         new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain),
//         new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy),
//         new SetShooterCommanded(mShooter, false)
//       )
//     );
//   }
// }
