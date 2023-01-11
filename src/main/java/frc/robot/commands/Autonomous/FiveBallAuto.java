// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.SetBottomLiftCommandedNew;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.SetShooterSpeedTargets;
import frc.robot.commands.SetTurretTargetAngle;
import frc.robot.commands.StartHood;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShootAutonomous;
import frc.robot.paths.FiveBallFinalPart1Path;
import frc.robot.paths.FiveBallFinalPart2Path;
import frc.robot.paths.ThreeBallForFivePath;
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
public class FiveBallAuto extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public FiveBallAuto(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret,
      CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel,
      BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
    addCommands(
        // new NewHoodTarget(mShooterHood, 0.0),
        // new StartHood(mShooterHood),
        // new SetShooterSpeedTargets(mShooter, 1500, 2000),

        new ResetGyro(mDrivetrain, 88.5),
        new NewHoodTarget(mShooterHood, -67.5),
        new StartHood(mShooterHood),
        new SetShooterSpeedTargets(mShooter, 1700, 2700),
        new SetShooterCommanded(mShooter, true),
        new AutoTurretAimAutonomous(mTurret, true, 194),
        // AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
        // new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
        new SequentialCommandGroup(
            // new WaitCommand(0.25),
            // new WaitCommand(0.040),
            new InstantCommand(mTurret::autoAimingOff),
            new AutoShootAutonomous(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain)
                .withTimeout(0.8),
            new InstantCommand(mTurret::autoAimingOn),
            new ParallelCommandGroup(
                new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
                new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
                new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
                new SequentialCommandGroup(
                    new ChangeIntakeState(mIntakeDeploy, true),
                    new SetBottomLiftCommandedNew(mBottomLift, false, false, 0.0, 0.0),
                    new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, true),
                    new NewHoodTarget(mShooterHood, 106),
                    new SetTurretTargetAngle(mTurret, true, 93.0),
                    new SetShooterSpeedTargets(mShooter, 2400, 2900),
                    new AutoFollowPath(mDrivetrain,
                        new ThreeBallForFivePath(mDrivetrain, 88.5).generateSwerveTrajectory(), true, false, 0.0)
                            .withTimeout(5),
                    new WaitCommand(0.20),
                    new AutoShootAutonomous(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain)
                        .withTimeout(1.3),
                    new SetBottomLiftCommandedNew(mBottomLift, false, true, 0.0, 0.0),
                    new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, true),
                    new SetTurretTargetAngle(mTurret, true, 181),
                    new SetShooterSpeedTargets(mShooter, 2800, 3150),
                    new NewHoodTarget(mShooterHood, 148),
                    new AutoFollowPath(mDrivetrain,
                        new FiveBallFinalPart1Path(mDrivetrain, -125).generateSwerveTrajectory(), false, false, -130.0)
                            .withTimeout(7.5),
                    new WaitCommand(0.6),
                    new AutoFollowPath(mDrivetrain,
                        new FiveBallFinalPart2Path(mDrivetrain, 135).generateSwerveTrajectory(), false, false, 135)
                            .withTimeout(7.5),
                    new WaitCommand(0.2),
                    new AutoShootAutonomous(mCargoFunnel, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain)
                        .withTimeout(3)))

        ));
  }
}
