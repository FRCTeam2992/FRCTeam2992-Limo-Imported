// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;

public class ClimbSticks extends CommandBase {
  //Subsystem
  private Climb mClimb;
  private Drivetrain mDrivetrain;

  private boolean traverseLockPressed = false;             // True when traverse lockk button press processed
  private boolean traverseLocked = false;                  // True when extension being held until at right angle
  private Debouncer climbOnDebounce;
  private MedianFilter pitchFilter;
  private MedianFilter picthChangeFilter;


  public ClimbSticks(Climb climb, Drivetrain drivetrain) {

    mClimb = climb;
    addRequirements(climb);
    mDrivetrain = drivetrain;
    climbOnDebounce = new Debouncer(0.05, DebounceType.kBoth);
    pitchFilter = new MedianFilter(5);
    picthChangeFilter = new MedianFilter(5);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchFilter.reset();
    picthChangeFilter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbY;
   
    climbY = -Robot.mRobotContainer.controller1.getLeftY();

    if (Math.abs(climbY) <= Constants.joystickDeadband * 2){
      climbY = 0.0;
    }      
    climbY = climbY * climbY * climbY;
    
    climbY = MathUtil.clamp(climbY, -1.0, 1.0);     // Temp clamp for testing
    
    // if
    // (climbOnDebounce.calculate(Robot.mRobotContainer.controller1.getRightStickButton()))
    // {
    // // Climb mode button pressed -- need to do interlock to time traverse
    // extension
    // if (!traverseLockPressed) {
    // // New button press, so set locked mode
    // traverseLockPressed = true;
    // traverseLocked = true;
    // } else {
    // // Not a new button press so change no state
    // }
      
    // // Update the current pitch angle and rate of change using filters
    // double pitch = pitchFilter.calculate(mDrivetrain.getLastPitch());
    // double pitchChange =
    // picthChangeFilter.calculate(mDrivetrain.getPitchChange());
      
    // // If "locked" and trying to extend climber make sure we are at right part of
    // swing
    // if (traverseLocked && (climbY > 0)) {
    // // We are traverse locked and trying to extend so need to check locks
    // if (((pitchChange < Constants.traversePitchMinDelta) && // Note pitchChange
    // must be negative
    // (pitch > Constants.traverseMinPitch) &&
    // (pitch < Constants.traverseMaxPitch)) ||
    // ((mClimb.getLeftEncoderAngle() > Constants.traverseLockMaxEncoder) &&
    // (mClimb.getRighttEncoderAngle() > Constants.traverseLockMaxEncoder))) {
    // // Swinging fast enough in the right direction and right part of swing OR we
    // have already released
    // traverseLocked = false; // Met criteria so undo the lock
    // } else {
    // // Still locked so zero the power to climb motors
    // climbY = 0.0;
    // }
        
    // } else {
    // // Change nothing -- Pressing button but already unlocked or not trying to
    // extend so allow
    // }

    // if ((climbY > 0) && (mClimb.getLeftEncoderAngle() > Constants.traversalPause)
    // &&
    // (mClimb.getRighttEncoderAngle() > Constants.traversalPause) &&
    // (pitch > Constants.traversalFullExtendPitch)) {
    // // Moving up and in pause zone and not yet sloped enough
    // //if (pitchChange < Constants.traversePitchMinDelta) {
    // climbY = 0.0;
      
    // }
    // } else {
    // // No longer pressing button so clear all traverse locks
    // traverseLockPressed = false;
    // traverseLocked = false;
    // pitchFilter.reset();
    // picthChangeFilter.reset();
    // }

    mClimb.setClimbSpeedSmart(climbY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}

