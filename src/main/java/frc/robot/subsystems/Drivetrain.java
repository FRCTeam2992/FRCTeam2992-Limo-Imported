// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.lib.drive.swerve.SwerveModuleNeoNeo;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Drive Motors
  private CANSparkMax frontLeftDrive;
  private CANSparkMax frontLeftTurn;

  private CANSparkMax frontRightDrive;
  private CANSparkMax frontRightTurn;

  private CANSparkMax rearLeftDrive;
  private CANSparkMax rearLeftTurn;

  private CANSparkMax rearRightDrive;
  private CANSparkMax rearRightTurn;

  // private WPI_TalonFX frontLeftDrive;
  // private WPI_TalonFX frontLeftTurn;

  // private WPI_TalonFX frontRightDrive;
  // private WPI_TalonFX frontRightTurn;

  // private WPI_TalonFX rearLeftDrive;
  // private WPI_TalonFX rearLeftTurn;

  // private WPI_TalonFX rearRightDrive;
  // private WPI_TalonFX rearRightTurn;

  // Swerve modules

  // Module CAN Encoders
  private final CANCoder frontLeftEncoder;
  private final CANCoder frontRightEncoder;
  private final CANCoder rearLeftEncoder;
  private final CANCoder rearRightEncoder;

  // Turn PID Controllers
  private final PIDController frontLeftController;
  private final PIDController frontRightController;
  private final PIDController rearLeftController;
  private final PIDController rearRightController;

  // Swerve Modules
  public final SwerveModuleNeoNeo frontLeftModule;
  public final SwerveModuleNeoNeo frontRightModule;
  public final SwerveModuleNeoNeo rearLeftModule;
  public final SwerveModuleNeoNeo rearRightModule;

  // Swerve Controller
  public final SwerveController swerveController;

  // Robot Gyro
  public AHRS navx;
  public double gyroOffset = 0.0;

  public Pose2d latestSwervePose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  public Pose2d priorSwervePose; // The pose from prior cycle
  public Pose2d latestSwervePoseEstimate = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  public Pose2d priorSwervePoseEstimate;
  private double distanceTraveled; // How far we moved this cycle (meters)
  public double angleTurned; // How much did we rotate this cycle (degrees)

  public boolean isPoseEstimation = false;

  // Swerve Drive Kinematics
  public final SwerveDriveKinematics swerveDriveKinematics;

  // Swerve Drive Odometry
  public final SwerveDriveOdometry swerveDriveOdometry;
  public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  public SwerveModulePosition[] swerveDriveModulePositions;

  public Transform2d moved;

  // // Swerve Pose

  // Motion Trajectories
  public Trajectory testPathTrajectory;
  public Trajectory threeBallMainTrajectory;
  public Trajectory fiveBallFinalTrajectory;
  public Trajectory twoBallTrajectory;
  public Trajectory threeBallForFiveTrajectory;
  public Trajectory fiveBallFinalPart1Trajectory;
  public Trajectory fiveBallFinalPart2Trajectory;

  // DriveTrain Dashboard Update Counter
  private int dashboardCounter = 0;

  // Slow mode flag
  private boolean inSlowMode = false;

  public Drivetrain() {
    // Drive Motors
    frontRightDrive = new CANSparkMax(2, MotorType.kBrushless);
    frontRightDrive.setInverted(false);
    // frontRightDrive = new WPI_TalonFX(2);
    // frontRightDrive.setInverted(false);
    setMotorCANPeriods(frontRightDrive);
    // addChild("frontRightDrive", frontRightDrive);

    frontRightTurn = new CANSparkMax(3, MotorType.kBrushless);
    frontRightTurn.setInverted(true);
    // frontRightTurn = new WPI_TalonFX(3);
    // frontRightTurn.setInverted(true);
    setMotorCANPeriods(frontRightTurn);
    // addChild("frontRightTurn", frontRightTurn);

    frontLeftDrive = new CANSparkMax(4, MotorType.kBrushless);
    frontLeftDrive.setInverted(false);
    // frontLeftDrive = new WPI_TalonFX(4);
    // frontLeftDrive.setInverted(false);
    setMotorCANPeriods(frontLeftDrive);
    // addChild("frontLeftDrive", frontLeftDrive);

    frontLeftTurn = new CANSparkMax(5, MotorType.kBrushless);
    frontLeftTurn.setInverted(true);
    // frontLeftTurn = new WPI_TalonFX(5);
    // frontLeftTurn.setInverted(true);
    setMotorCANPeriods(frontLeftTurn);
    // addChild("frontLeftTurn", frontLeftTurn);

    rearRightDrive = new CANSparkMax(6, MotorType.kBrushless);
    rearRightDrive.setInverted(false);
    // rearRightDrive = new WPI_TalonFX(6);
    // rearRightDrive.setInverted(false);
    setMotorCANPeriods(rearRightDrive);
    // addChild("rearRightDrive", rearRightDrive);

    rearRightTurn = new CANSparkMax(7, MotorType.kBrushless);
    rearRightTurn.setInverted(true);
    // rearRightTurn = new WPI_TalonFX(7);
    // rearRightTurn.setInverted(true);
    setMotorCANPeriods(rearRightTurn);
    // addChild("rearRightTurn", rearRightTurn);

    rearLeftDrive = new CANSparkMax(8, MotorType.kBrushless);
    rearLeftDrive.setInverted(false);
    // rearLeftDrive = new WPI_TalonFX(8);
    // rearLeftDrive.setInverted(false);
    setMotorCANPeriods(rearLeftDrive);
    // addChild("rearLeftDrive", rearLeftDrive);

    rearLeftTurn = new CANSparkMax(9, MotorType.kBrushless);
    rearLeftTurn.setInverted(true);
    // rearLeftTurn = new WPI_TalonFX(9);
    // rearLeftTurn.setInverted(true);
    setMotorCANPeriods(rearLeftTurn);
    // addChild("rearLeftTurn", rearLeftTurn);

    // Set motor states
    setDriveNeutralMode(IdleMode.kCoast);
    setTurnNeutralMode(IdleMode.kBrake);

    setDriveCurrentLimit(40, 60);
    setTurnCurrentLimit(30, 60);

    // Drive Encoders
    frontRightEncoder = new CANCoder(3);
    frontRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    frontRightEncoder.configSensorDirection(true);

    frontLeftEncoder = new CANCoder(5);
    frontLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    frontLeftEncoder.configSensorDirection(true);

    rearRightEncoder = new CANCoder(7);
    rearRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearRightEncoder.configSensorDirection(true);

    rearLeftEncoder = new CANCoder(9);
    rearLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearLeftEncoder.configSensorDirection(true);

    // Turn PID Controllers
    frontLeftController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    frontLeftController.enableContinuousInput(-180.0, 180.0);
    frontLeftController.setTolerance(2.0);

    frontRightController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    frontRightController.enableContinuousInput(-180.0, 180.0);
    frontRightController.setTolerance(2.0);

    rearLeftController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    rearLeftController.enableContinuousInput(-180.0, 180.0);
    rearLeftController.setTolerance(2.0);

    rearRightController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    rearRightController.enableContinuousInput(-180.0, 180.0);
    rearRightController.setTolerance(2.0);

    // Set the Drive PID Controllers
    frontLeftDrive.getPIDController().setP(Constants.driveP);
    frontLeftDrive.getPIDController().setI(Constants.driveI);
    frontLeftDrive.getPIDController().setD(Constants.driveD);
    frontLeftDrive.getPIDController().setFF(Constants.driveF);
    // frontLeftDrive.config_kP(0, Constants.driveP);
    // frontLeftDrive.config_kI(0, Constants.driveI);
    // frontLeftDrive.config_kD(0, Constants.driveD);
    // frontLeftDrive.config_kF(0, Constants.driveF);

    frontRightDrive.getPIDController().setP(Constants.driveP);
    frontRightDrive.getPIDController().setI(Constants.driveI);
    frontRightDrive.getPIDController().setD(Constants.driveD);
    frontRightDrive.getPIDController().setFF(Constants.driveF);

    // frontRightDrive.config_kP(0, Constants.driveP);
    // frontRightDrive.config_kI(0, Constants.driveI);
    // frontRightDrive.config_kD(0, Constants.driveD);
    // frontRightDrive.config_kF(0, Constants.driveF);

    rearLeftDrive.getPIDController().setP(Constants.driveP);
    rearLeftDrive.getPIDController().setI(Constants.driveI);
    rearLeftDrive.getPIDController().setD(Constants.driveD);
    rearLeftDrive.getPIDController().setFF(Constants.driveF);

    // rearLeftDrive.config_kP(0, Constants.driveP);
    // rearLeftDrive.config_kI(0, Constants.driveI);
    // rearLeftDrive.config_kD(0, Constants.driveD);
    // rearLeftDrive.config_kF(0, Constants.driveF);

    rearRightDrive.getPIDController().setP(Constants.driveP);
    rearRightDrive.getPIDController().setI(Constants.driveI);
    rearRightDrive.getPIDController().setD(Constants.driveD);
    rearRightDrive.getPIDController().setFF(Constants.driveF);

    // rearRightDrive.config_kP(0, Constants.driveP);
    // rearRightDrive.config_kI(0, Constants.driveI);
    // rearRightDrive.config_kD(0, Constants.driveD);
    // rearRightDrive.config_kF(0, Constants.driveF);

    // Swerve Modules
    // Added New/Neo swerve objects
    frontLeftModule = new SwerveModuleNeoNeo(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
            Constants.frontLeftOffset, frontLeftController, Constants.driveWheelDiameter, Constants.driveGearRatio,
            Constants.swerveMaxSpeed);

    frontRightModule = new SwerveModuleNeoNeo(frontRightDrive, frontRightTurn, frontRightEncoder,
            Constants.frontRightOffset, frontRightController, Constants.driveWheelDiameter, Constants.driveGearRatio,
            Constants.swerveMaxSpeed);

    rearLeftModule = new SwerveModuleNeoNeo(rearLeftDrive, rearLeftTurn, rearLeftEncoder,
            Constants.rearLeftOffset,
            rearLeftController, Constants.driveWheelDiameter, Constants.driveGearRatio, Constants.swerveMaxSpeed);

    rearRightModule = new SwerveModuleNeoNeo(rearRightDrive, rearRightTurn, rearRightEncoder,
            Constants.rearRightOffset, rearRightController, Constants.driveWheelDiameter, Constants.driveGearRatio,
            Constants.swerveMaxSpeed);

    // Removed Falcon/Falcon swerve objectsdd
    // frontLeftModule = new SwerveModuleFalconFalcon(frontLeftDrive, frontLeftTurn,
    // frontLeftEncoder,
    // Constants.frontLeftOffset, frontLeftController, Constants.driveWheelDiameter,
    // Constants.driveGearRatio,
    // Constants.swerveMaxSpeed);

    // frontRightModule = new SwerveModuleFalconFalcon(frontRightDrive,
    // frontRightTurn, frontRightEncoder,
    // Constants.frontRightOffset, frontRightController,
    // Constants.driveWheelDiameter, Constants.driveGearRatio,
    // Constants.swerveMaxSpeed);

    // rearLeftModule = new SwerveModuleFalconFalcon(rearLeftDrive, rearLeftTurn,
    // rearLeftEncoder,
    // Constants.rearLeftOffset,
    // rearLeftController, Constants.driveWheelDiameter, Constants.driveGearRatio,
    // Constants.swerveMaxSpeed);

    // rearRightModule = new SwerveModuleFalconFalcon(rearRightDrive, rearRightTurn,
    // rearRightEncoder,
    // Constants.rearRightOffset, rearRightController, Constants.driveWheelDiameter,
    // Constants.driveGearRatio,
    // Constants.swerveMaxSpeed);

    // Swerve Controller
    swerveController = new SwerveController(Constants.swerveLength, Constants.swerveWidth);
    swerveDriveModulePositions = new SwerveModulePosition[4];
    swerveDriveModulePositions[0] = frontLeftModule.getPosition();
    swerveDriveModulePositions[1] = frontRightModule.getPosition();
    swerveDriveModulePositions[2] = rearLeftModule.getPosition();
    swerveDriveModulePositions[3] = rearRightModule.getPosition();

    // robot gyro initialization
    navx = new AHRS(SPI.Port.kMXP);

    // Swerve Drive Kinematics
    swerveDriveKinematics = new SwerveDriveKinematics(Constants.frontLeftLocation,
        Constants.frontRightLocation,
        Constants.rearLeftLocation, Constants.rearRightLocation);

    // Swerve Drive Position array

    // Serve Drive Odometry
    swerveDriveOdometry = new SwerveDriveOdometry(
        swerveDriveKinematics, Rotation2d.fromDegrees(navx.getYaw()),
        swerveDriveModulePositions,
        new Pose2d(0.0, 0.0, new Rotation2d()));

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        swerveDriveKinematics, Rotation2d.fromDegrees(navx.getYaw()),
        swerveDriveModulePositions, new Pose2d(0.0, 0.0, new Rotation2d()));
    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State
    // measurement standard deviations. X, Y, theta.
    // new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), // Local measurement
    // standard deviations. Left encoder, right encoder, gyro.
    // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.05)); // Global
    // measurement standard deviations. X, Y, and theta.

    // Motion Trajectories
    loadMotionPaths();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DriveTrain Dashboard Update
    if (++dashboardCounter >= 5) {
      // Display LimeLight Distance to Target
      // SmartDashboard.putNumber("Limelight Distance",
      // limeLightCamera.getDistanceToTarget(Constants.cameraAngle,
      // Constants.cameraHeight, Constants.targetHeight));

      // Display Gyro Angle
      // SmartDashboard.putNumber("Gyro Yaw", navx.getYaw());

      // // Display Module Angles
      SmartDashboard.putNumber("Front Left Module Angle",
              frontLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Front Right Module Angle",
              frontRightModule.getEncoderAngle());
      SmartDashboard.putNumber("Rear Left Module Angle",
              rearLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Rear Right Module Angle",
              rearRightModule.getEncoderAngle());

      // Display Wheel Velocities
      // SmartDashboard.putNumber("Front Left Module Velocity",
      // frontLeftModule.getWheelSpeedMeters());
      // SmartDashboard.putNumber("Front Right Module Velocity",
      // frontRightModule.getWheelSpeedMeters());
      // SmartDashboard.putNumber("Rear Left Module Velocity",
      // rearLeftModule.getWheelSpeedMeters());
      // SmartDashboard.putNumber("Rear Right Module Velocity",
      // rearRightModule.getWheelSpeedMeters());

      dashboardCounter = 0;

      // SmartDashboard.putNumber("Gyro Pitch", navx.getPitch());
      // SmartDashboard.putNumber("Gyro Roll", navx.getRoll());
      // SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
      // SmartDashboard.putBoolean("Gyro Ready", navx.isConnected());
      // SmartDashboard.putBoolean("Gyro Calibrating", navx.isCalibrating());

      // SmartDashboard.putBoolean("Is In Slow Mode", isInSlowMode());
    }

    // Update the Odometry
    // if (DriverStation.isAutonomous()) {
    priorSwervePose = latestSwervePose;
    latestSwervePose = swerveDriveOdometry.update(
        Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions);
    moved = latestSwervePose.minus(priorSwervePose);
    // }
    // else {

    // priorSwervePoseEstimate = latestSwervePoseEstimate;
    // latestSwervePoseEstimate = swerveDrivePoseEstimator.updateWithTime(
    // Timer.getFPGATimestamp(), Rotation2d.fromDegrees(-getGyroYaw()),
    // frontLeftModule.getState(), frontRightModule.getState(),
    // rearLeftModule.getState(), rearRightModule.getState());

    // moved = latestSwervePoseEstimate.minus(priorSwervePoseEstimate);
    // }

    distanceTraveled = Math.sqrt(moved.getX() * moved.getX() + moved.getY() * moved.getY());
    angleTurned = Math.abs(moved.getRotation().getDegrees());

    // Display Odometry
    // SmartDashboard.putNumber("Odometry Rotation",
    // latestSwervePose.getRotation().getDegrees());
    // SmartDashboard.putNumber("Odometry X", (latestSwervePose.getX() * (100 /
    // 2.54)));
    // SmartDashboard.putNumber("Odometry Y", (latestSwervePose.getY() * (100 /
    // 2.54)));

    // SmartDashboard.putNumber("Estimation Rotation",
    // latestSwervePoseEstimate.getRotation().getDegrees());
    // SmartDashboard.putNumber("Estimation X", latestSwervePoseEstimate.getX() *
    // (100 / 2.54));
    // SmartDashboard.putNumber("Estimation Y", latestSwervePoseEstimate.getY() *
    // (100 / 2.54));

    // Update the pitch info
    // pitchChange = navx.getPitch() - lastPitch;
    // lastPitch = navx.getPitch();
  }

  public void setDriveNeutralMode(IdleMode mode) {
      frontLeftDrive.setIdleMode(mode);
      frontRightDrive.setIdleMode(mode);
      rearLeftDrive.setIdleMode(mode);
      rearRightDrive.setIdleMode(mode);
  }

  public void setTurnNeutralMode(IdleMode mode) {
      frontLeftTurn.setIdleMode(mode);
      frontRightTurn.setIdleMode(mode);
      rearLeftTurn.setIdleMode(mode);
      rearRightTurn.setIdleMode(mode);
  }

  public void setDriveCurrentLimit(int currentLimit, int triggerCurrent) {
      frontLeftDrive.setSmartCurrentLimit(currentLimit);
      frontLeftDrive.setSecondaryCurrentLimit(triggerCurrent);
      frontRightDrive.setSmartCurrentLimit(currentLimit);
      frontRightDrive.setSecondaryCurrentLimit(triggerCurrent);
      rearLeftDrive.setSmartCurrentLimit(currentLimit);
      rearLeftDrive.setSecondaryCurrentLimit(triggerCurrent);
      rearRightDrive.setSmartCurrentLimit(currentLimit);
      rearRightDrive.setSecondaryCurrentLimit(triggerCurrent);

      // frontLeftDrive.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
      // frontRightDrive
      // .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
      // currentLimit, triggerCurrent, 0));
      // rearLeftDrive.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
      // rearRightDrive.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, currentLimit, triggerCurrent, 0));
  }

  // seconds from idle to max speed
  public void setDriveRampRate(double seconds) {
    // Open loop ramp rates
    frontLeftDrive.setOpenLoopRampRate(seconds);
    frontRightDrive.setOpenLoopRampRate(seconds);
    rearLeftDrive.setOpenLoopRampRate(seconds);
    rearRightDrive.setOpenLoopRampRate(seconds);

    // Closed loop ramp rates
    frontLeftDrive.setClosedLoopRampRate(seconds);
    frontRightDrive.setClosedLoopRampRate(seconds);
    rearLeftDrive.setClosedLoopRampRate(seconds);
    rearRightDrive.setClosedLoopRampRate(seconds);
  }

  public void setTurnCurrentLimit(int currentLimit, int triggerCurrent) {
      frontLeftTurn.setSmartCurrentLimit(currentLimit);
      frontLeftTurn.setSecondaryCurrentLimit(triggerCurrent);
      frontRightTurn.setSmartCurrentLimit(currentLimit);
      frontRightTurn.setSecondaryCurrentLimit(triggerCurrent);
      rearLeftTurn.setSmartCurrentLimit(currentLimit);
      rearLeftTurn.setSecondaryCurrentLimit(triggerCurrent);
      rearRightTurn.setSmartCurrentLimit(currentLimit);
      rearRightTurn.setSecondaryCurrentLimit(triggerCurrent);

      // frontLeftTurn.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, current, current, 0));
      // frontRightTurn.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, current, current, 0));
      // rearLeftTurn.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, current, current, 0));
      // rearRightTurn.configSupplyCurrentLimit(new
      // SupplyCurrentLimitConfiguration(true, current, current, 0));
  }

  public void stopDrive() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void resetOdometry() {
    swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
        new Pose2d(0.0, 0.0, new Rotation2d()));
    // swerveDrivePoseEstimator.resetPosition(new Pose2d(0.0, 0.0, new
    // Rotation2d()),
    // Rotation2d.fromDegrees(-getGyroYaw()));
  }

  public void resetPoseEstimate() {
    // swerveDriveOdometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d()),
    // Rotation2d.fromDegrees(-getGyroYaw()));
    swerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions,
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public void setOdometryPosition(boolean useGyro, Pose2d position) {
    swerveDriveOdometry.resetPosition(
        useGyro ? Rotation2d.fromDegrees(-getGyroYaw()) : Rotation2d.fromDegrees(0.0), swerveDriveModulePositions,
        position);

    latestSwervePose = swerveDriveOdometry.update(Rotation2d.fromDegrees(-getGyroYaw()), swerveDriveModulePositions);

    // swerveDrivePoseEstimator.resetPosition(position,
    // useGyro?Rotation2d.fromDegrees(-getGyroYaw()):Rotation2d.fromDegrees(0.0));

    // latestSwervePoseEstimate =
    // swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getGyroYaw()),
    // frontLeftModule.getState(),
    // frontRightModule.getState(), rearLeftModule.getState(),
    // rearRightModule.getState());
  }

  public void setPoseEstimatePosition(boolean useGyro, Pose2d position) {
    // swerveDriveOdometry.resetPosition(position,
    // useGyro?Rotation2d.fromDegrees(-getGyroYaw()):Rotation2d.fromDegrees(0.0));

    // latestSwervePose =
    // swerveDriveOdometry.update(Rotation2d.fromDegrees(-getGyroYaw()),
    // frontLeftModule.getState(),
    // frontRightModule.getState(), rearLeftModule.getState(),
    // rearRightModule.getState());

    swerveDrivePoseEstimator.resetPosition(
        useGyro ? Rotation2d.fromDegrees(-getGyroYaw()) : Rotation2d.fromDegrees(0.0), swerveDriveModulePositions,
        position);

    latestSwervePoseEstimate = swerveDrivePoseEstimator.update(Rotation2d.fromDegrees(-getGyroYaw()),
        swerveDriveModulePositions);
  }

  public double getGyroYaw() {
    double angle = navx.getYaw() + gyroOffset;
    while (angle > 360) {
      angle -= 360;
    }
    while (angle < 0) {
      angle += 360;
    }
    return angle;
  }

  private void loadMotionPaths() {
    // Trajectory Paths
    Path testPath = Filesystem.getDeployDirectory().toPath().resolve("output/TestPath.wpilib.json");
    Path threeBallPathMain = Filesystem.getDeployDirectory().toPath().resolve("output/ThreeBallPathMain.wpilib.json");
    Path fiveBallPath = Filesystem.getDeployDirectory().toPath().resolve("output/FiveBallPathFinal.wpilib.json");
    Path twoBallPath = Filesystem.getDeployDirectory().toPath().resolve("output/TwoBallPath.wpilib.json");
    Path threeBallPathForFive = Filesystem.getDeployDirectory().toPath()
        .resolve("output/ThreeBallPathForFive.wpilib.json");
    Path fiveBallPathPart1 = Filesystem.getDeployDirectory().toPath().resolve("output/FiveBallPathPart1.wpilib.json");
    Path fiveBallPathPart2 = Filesystem.getDeployDirectory().toPath().resolve("output/FiveBallPathPart2.wpilib.json");

    try {
      testPathTrajectory = TrajectoryUtil.fromPathweaverJson(testPath);
      threeBallMainTrajectory = TrajectoryUtil.fromPathweaverJson(threeBallPathMain);
      fiveBallFinalTrajectory = TrajectoryUtil.fromPathweaverJson(fiveBallPath);
      twoBallTrajectory = TrajectoryUtil.fromPathweaverJson(twoBallPath);
      threeBallForFiveTrajectory = TrajectoryUtil.fromPathweaverJson(threeBallPathForFive);
      fiveBallFinalPart1Trajectory = TrajectoryUtil.fromPathweaverJson(fiveBallPathPart1);
      fiveBallFinalPart2Trajectory = TrajectoryUtil.fromPathweaverJson(fiveBallPathPart2);
    } catch (IOException e) {
      DriverStation.reportError("Unable to load motion trajectories!", e.getStackTrace());
      e.printStackTrace();
    }

  }

  private void setMotorCANPeriods(CANSparkMax motor) {
      motor.setControlFramePeriodMs(20);
      // motor.setStatusFramePeriod(1, 100); // Applied Motor Output
      // // Don't change frame type 2 -- Selected Sensor position needed for Odometry
      // // Don't change frame type 3 -- Quadrature info -- is needed?
      // motor.setStatusFramePeriod(4, 255); // Analog and battery voltage info
      // motor.setStatusFramePeriod(8, 254); // PWM Info not used
  }

  public double getDistanceTraveled() {
    return distanceTraveled;
  }

  public double getAngleTurned() {
    return angleTurned;
  }

  public boolean isInSlowMode() {
    return inSlowMode;
  }

  public void setInSlowMode(boolean inSlowMode) {
    this.inSlowMode = inSlowMode;
  }

}
