// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BottomLift extends SubsystemBase {

  private WPI_TalonSRX bottomLiftMotor;

  private boolean isCommanded = false; // Is commanded to be moving during default command
  private boolean checkSensor = false; // CHeck the ball sensor if commanded
 
  private double noBallSpeed = 0.0; // Speed to be commanded if no ball is seen
  private double withBallSpeed = 0.0; // Speed to be commanded if we see a ball
  private double sensorDelay = 0.0; // How long after seeing a ball before we jump to the withBallSpeed
  private boolean limitBypassed = false;
  private boolean hasTopBall = false;
  private double encoderTarget = 0.0;
  // private double encoderHoldPosition = 0.0;
  // private Encoder liftTowerEncoder;

  private Timer sensorTimer;
  // private Timer turnOnTimer;

  private DigitalInput bottomLiftSensor;
  // private DigitalInput topLiftSensor;

  private double dashboardCounter = 0;

  private Debouncer topSensorDebouncer;
  private Debouncer bottomSensorDebouncer;

  private DataLog mDataLog;
  private BooleanLogEntry bottomSensorLog;
  private BooleanLogEntry topSensorLog;
  private StringLogEntry commandedBallSpeedLog;
  private BooleanLogEntry bottomSensorDebouncedLog;
  private BooleanLogEntry topSensorDebouncedLog;
  private DoubleLogEntry sensorTimerLog;

  public boolean updatedBottomSensorState;
  public boolean updatedTopSensorState;

  public BottomLift() {
    bottomLiftMotor = new WPI_TalonSRX(24);
    configLiftMotor();

    addChild("bottomLiftMotor", bottomLiftMotor);

    bottomLiftSensor = new DigitalInput(0);
    // topLiftSensor = new DigitalInput(1);

    topSensorDebouncer = new Debouncer(.1, DebounceType.kFalling);
    bottomSensorDebouncer = new Debouncer(.1, DebounceType.kFalling);

    // liftTowerEncoder = new Encoder(3, 4, false, EncodingType.k4X);

    sensorTimer = new Timer();
    sensorTimer.start();
    sensorTimer.reset();

    // turnOnTimer = new Timer();
    // turnOnTimer.start();
    // turnOnTimer.reset();

    if (Constants.dataLogging) {
      mDataLog = DataLogManager.getLog();
      bottomSensorLog = new BooleanLogEntry(mDataLog, "/bl/bottomSensor");
      topSensorLog = new BooleanLogEntry(mDataLog, "/bl/topSensor");
      commandedBallSpeedLog = new StringLogEntry(mDataLog, "/bl/commandedBallSpeed");
      bottomSensorDebouncedLog = new BooleanLogEntry(mDataLog, "/bl/bottomSensorDebounced");
      topSensorDebouncedLog = new BooleanLogEntry(mDataLog, "/bl/topSensorDebounced");
      sensorTimerLog = new DoubleLogEntry(mDataLog, "/bl/timer");
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("bottom sensor", getBottomSensorState());
    // SmartDashboard.putBoolean("top sensor", getTopSensorState());
    if (Constants.dataLogging) {
      //topSensorLog.append(updatedTopSensorState);
      //bottomSensorLog.append(updatedBottomSensorState);
        bottomSensorLog.append(getBottomSensorState());
        bottomSensorDebouncedLog.append(updatedBottomSensorState);
        topSensorLog.append(getTopSensorState());
        topSensorDebouncedLog.append(updatedTopSensorState);
        sensorTimerLog.append(sensorTimer.get());
    }
    if (dashboardCounter++ > 5) {
      // SmartDashboard.putBoolean("top sensor", updatedTopSensorState);
      // SmartDashboard.putBoolean("bottom sensor", updatedBottomSensorState);
      // SmartDashboard.putNumber("Bottom Lift Encoder", bottomLiftMotor.getSelectedSensorPosition());
      // SmartDashboard.putString("bottom lift control mode", bottomLiftMotor.getControlMode().toString());
      // SmartDashboard.putNumber("bottom lift motor power", bottomLiftMotor.getMotorOutputPercent());

      if (bottomLiftMotor.hasResetOccurred()) {
        configLiftMotor();
      }
    }

  }

  public void setBottomLiftSpeed(double speed) {
    if (!limitBypassed) {
      limitBypassed = true;
      bottomLiftMotor.overrideLimitSwitchesEnable(false);
    }
    hasTopBall = false;
    bottomLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  /*
   * public void setBottomLiftSpeedAsCommandedSensor() {
   * if (isCommanded()) {
   * // We should be running in default command
   * sensorTimer.reset();
   * if ((!getBottomSensorState() && !getTopSensorState()) || (sensorTimer.get() <
   * getSensorDelay())) {
   * // No ball is seen so run at higher speed
   * bottomLiftMotor.set(ControlMode.PercentOutput, noBallSpeed);
   * } else {
   * // We have a ball in cargo lift for long enough so change speed
   * bottomLiftMotor.set(ControlMode.PercentOutput, withBallSpeed);
   * }
   * } else {
   * // We are commanded off
   * bottomLiftMotor.set(ControlMode.PercentOutput, 0.0);
   * }
   * }
   */
  public void setBottomLiftSpeedAsCommandedSensorNew() {
    
    if (isCommanded()) {
      if (!checkSensor) {
        if (!limitBypassed) {
          limitBypassed = true;
          bottomLiftMotor.overrideLimitSwitchesEnable(false);
        }
        hasTopBall = false;
        bottomLiftMotor.set(ControlMode.PercentOutput, noBallSpeed);

      } else if (updatedTopSensorState) {
        if (!limitBypassed && !hasTopBall) {
          limitBypassed = true;
          bottomLiftMotor.overrideLimitSwitchesEnable(false);
        }
        // If the top and the bottom or just the top is triggered
        if (!hasTopBall) {
          hasTopBall = true;
          // SmartDashboard.putNumber("initial encoder", bottomLiftMotor.getSelectedSensorPosition());
          encoderTarget = bottomLiftMotor.getSelectedSensorPosition() + Constants.liftEncoderBallAdvanceClicks;
          // SmartDashboard.putNumber("encoder target", encoderTarget);

          bottomLiftMotor.set(ControlMode.Position, encoderTarget);
        }

        if (Constants.dataLogging) {
          commandedBallSpeedLog.append("top sees ball");

        }

      } else if (updatedBottomSensorState) {
        // The bottom sees a ball run at lower speed
        hasTopBall = false;
        if (limitBypassed) {
          limitBypassed = false;
          bottomLiftMotor.overrideLimitSwitchesEnable(true);
        }
        bottomLiftMotor.set(ControlMode.PercentOutput, withBallSpeed);


        if (Constants.dataLogging) {
          commandedBallSpeedLog.append("bottom sees ball");
        }

      } else {
        // No ball is seen so run at higher speed
        hasTopBall = false;
        if (limitBypassed) {
          limitBypassed = false;
          bottomLiftMotor.overrideLimitSwitchesEnable(true);
        }

        bottomLiftMotor.set(ControlMode.PercentOutput, noBallSpeed);

        if (Constants.dataLogging) {
          commandedBallSpeedLog.append("neither sees ball");
        }
      }

    } else {
      bottomLiftMotor.set(ControlMode.PercentOutput, 0.0);
      hasTopBall = false;
      if (limitBypassed) {
        limitBypassed = false;
        bottomLiftMotor.overrideLimitSwitchesEnable(true);
      }

      if (Constants.dataLogging) {
        commandedBallSpeedLog.append("bottom lift is off");
      }
    }

  }

  public void fastPeriodic() {
    updatedBottomSensorState = bottomSensorDebouncer.calculate(getBottomSensorState());
    updatedTopSensorState = topSensorDebouncer.calculate(getTopSensorState());

    if (!updatedTopSensorState) {
      sensorTimer.reset();
    }
    
  }

  public boolean getBottomSensorState() {
    return bottomLiftSensor.get();
  }

  public boolean getTopSensorState() {
    // return topLiftSensor.get();
    return (bottomLiftMotor.isFwdLimitSwitchClosed() == 0);
  }

  public boolean isCommanded() {
    return isCommanded;
  }

  public void setCommanded(boolean isCommanded) {
    this.isCommanded = isCommanded;
  }

  public boolean isCheckSensor() {
    return checkSensor;
  }

  public void setCheckSensor(boolean checkSensor) {
    this.checkSensor = checkSensor;
  }

  public double getNoBallSpeed() {
    return noBallSpeed;
  }

  public void setNoBallSpeed(double noBallSpeed) {
    this.noBallSpeed = noBallSpeed;
  }

  public double getWithBallSpeed() {
    return withBallSpeed;
  }

  public void setWithBallSpeed(double withBallSpeed) {
    this.withBallSpeed = withBallSpeed;
  }

  public double getSensorDelay() {
    return sensorDelay;
  }

  public void setSensorDelay(double sensorDelay) {
    this.sensorDelay = sensorDelay;
  }

  public void reset() {
    setCommanded(false);
    setCheckSensor(false);
    setNoBallSpeed(0.0);
    setWithBallSpeed(0.0);
    setSensorDelay(0.0);
    hasTopBall = false;
    limitBypassed = false;
    bottomLiftMotor.overrideLimitSwitchesEnable(true);
  }

  public void configLiftMotor() {
    bottomLiftMotor.setInverted(false);
    bottomLiftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyClosed);
    bottomLiftMotor.setNeutralMode(NeutralMode.Brake);
    bottomLiftMotor.setSensorPhase(true);
    bottomLiftMotor.setStatusFramePeriod(1, 255);
    bottomLiftMotor.setStatusFramePeriod(2, 254);
    bottomLiftMotor.setStatusFramePeriod(3, 253);
    bottomLiftMotor.setStatusFramePeriod(4, 252);
    bottomLiftMotor.setStatusFramePeriod(8, 251);
    bottomLiftMotor.setStatusFramePeriod(10, 250);
    bottomLiftMotor.setStatusFramePeriod(12, 249);
    bottomLiftMotor.setStatusFramePeriod(13, 248);
    bottomLiftMotor.setStatusFramePeriod(14, 247);
    bottomLiftMotor.setStatusFramePeriod(21, 246);

  }
}
