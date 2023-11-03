
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// import org.ejml.FancyPrint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

  // Robot Period
  public static final double robotPeriod = 0.020;     // 20ms cycles

  // Single vs Double Controller mode
  // True = single controller mode
  public static final boolean isSingleController = true;

  //Intake Deploy Constants
  public static final double maxIntakeEncoderAngle = 50;
  public static final double minIntakeEncoderAngle = 0;
  
  public static final double intakeP = 0.1;
  public static final double intakeI = 0;
  public static final double intakeD = 0;
  public static final double intakeF = 0.005;
  public static final double intakeTolerance = 0;

  //Lift Tower
  public static final double liftEncoderClicksDeadBand = 100;
  public static final double liftEncoderBallAdvanceClicks = 5600; 
  //^^^ how far to turn the motor in order to position top ball in lift tower ^^^

  // Shooter Constants
  public static final double defaultMainShooterSpeed = 2700;
  public static final double defaultSecondaryShooterSpeed = 3300;
  public static final int shooterEncoderPulses = 2048;
  public static final double shooterSpeedTolerance = 100;
  
  public static final double shooterPIDMainP = 0.1;
  public static final double shooterPIDMainI = 0.0;
  public static final double shooterPIDMainD = 1.5;
  public static final double shooterPIDMainF = 0.055;

  public static final double shooterPIDSecondaryP = 0.1;
  public static final double shooterPIDSecondaryI = 0.0;
  public static final double shooterPIDSecondaryD = 1.5;
  public static final double shooterPIDSecondaryF = 0.052;

  // Turret Constants
  public static final double turretP = 0.06;
  public static final double turretI = 0.012;
  public static final double turretD = 1.5;
  public static final double turretF = -0.02; // Need to push against CF spring in neg dir
  
  public static final double turretTolerance = 1.0;
  public static final int turretEncoderOffset = 90;
  public static final int turretRobotOffset = 70; 
  public static final int turretMinEnd = 35; // 45
  public static final int turretMaxEnd = 320; // 298
  public static final int turretMinRumble = 50; // 60
  public static final int turretMaxRumble = 305; // 283
  public static final int turretMaxSlowZone = 245; // 223
  public static final int turretMinSlowZone = 110; // 100
  public static final double turretJoystickDeadband = .75;
  public static final double turretDefaultAngle = 180;
  public static final double turretGearRatio = (240.0 / 18.0) * (24.0 / 18.0);
  // public static final double turretGearRatio = 40.0 / 2.3;  // Faked to make angles work!

  // Vision Constants

  // the tooth to tooth of the hood 
  public static final double hoodAngleRatio = 38.000 / 520.000;
  public static final double hoodEncoderAngleRatio = 520.000 / 38.000;
  // the max and min of the encoder values not the hood angles
  public static final double minHoodPosition = -153.0;
  public static final double minHoodPZone = -100.0;

  public static final double maxHoodPosition = 153.0;
  public static final double maxHoodPZone = 100.0;

  public static final double hoodPValueBottom = 0.0045;
  public static final double hoodPValueTop = 0.0072;
  public static final double hoodEncoderOffset = -28.9 + 180;

  public static final double cameraHeight = 41.25;
  public static final double goalHeight = 102;
  public static final double distanceTest = 120;
  public static final double cameraAngle = 35;

  public static final double hoodP = .007;
  public static final double hoodI = 0.02; 
  public static final double hoodD = 0.0001;
  public static final double hoodF = 0.02; 

  public static final double hoodTolerance = 10;

  public static final double defaultHoodPosition = 0;

  
  // Field Geometry for Pose Esimation
  public static double goalX = 8.23;          // Meters
  public static double goalY = goalX / 2.0;   // Meters
  public static double goalRadius = .6096;     // Meters
  public static double limeLightOffset = 0.295;  // 11.593 inches = .295 meters
  public static double turretOffset = 0.0699;   // 2.75 inches = 0.0699 meters
  public static Pose2d goalPose = new Pose2d(goalX, goalY, new Rotation2d(0.0));

  // Drive Variables
  public static final boolean isFieldCentric = true;
  public static final boolean isVelocityControlled = true;
  public static final boolean isGyroCorrected = true;
  public static final double joystickDeadband = 0.15;
  public static double joystickXYSmoothFactor = 0.5;
  public static double joystickRotationSmoothFactor = 0.5;
  public static double joystickRotationInverseDeadband = 0.14;

  // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
  public static final double swerveWidth = 0.591;
  public static final double swerveLength = 0.654;

  // Max Swerve Speed (Velocity Control)
  public static final double swerveMaxSpeed = 4.5; // (Meters per Second)(2 Slow, 4.5 normal)

  // Swerve Wheels and Gear Ratio
  public static final double driveGearRatio = 6.75;// 6.75:1
  public static final double driveWheelDiameter = 0.098552;

  // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
  // Right
  public static final double frontLeftOffset = 147.65; // 146.42578125;
  public static final double frontRightOffset = -78.31; // -144.140625;
  public static final double rearLeftOffset = -179.91; // 101.25;
  public static final double rearRightOffset = 123.92; // -24.521484375;

  // Swerve Drive PID (Velocity Control)
  public static final double driveP = 0.05;
  public static final double driveI = 0.0;
  public static final double driveD = 0.01;
  public static final double driveF = 0.047;

  // Swerve Turn PIDs
  public static final double turnP = 0.0045; //.013
  public static final double turnI = 0.0;
  public static final double turnD = 0.00005;
  
  

  // Gyro P
  public static final double driveGyroP = 0.005;

  // Swerve Module Translations x=.591/2 y=.654/2
  public static final Translation2d frontLeftLocation = new Translation2d(0.2955, 0.327);
  public static final Translation2d frontRightLocation = new Translation2d(0.2955, -0.327);
  public static final Translation2d rearLeftLocation = new Translation2d(-0.2955, 0.327);
  public static final Translation2d rearRightLocation = new Translation2d(-0.2955, -0.327);

  // Swerve X Axis Correction PID (Path Following)
  public static final double xCorrectionP = 5.0;
  public static final double xCorrectionI = 0.0;
  public static final double xCorrectionD = 0.0;

  // Swerve Y Axis Correction PID (Path Following)
  public static final double yCorrectionP = 5.0;
  public static final double yCorrectionI = 0.0;
  public static final double yCorrectionD = 0.0;

  // Swerve Theta Axis Correction PID (Path Following)
  public static final double thetaCorrectionP = 3.0;
  public static final double thetaCorrectionI = 0.0;
  public static final double thetaCorrectionD = 0.0;

  // Max Path Following Drive Speeds
  public static final double maxPathFollowingVelocity = 3.0; // (Meters per Second)
  public static final double maxPathFollowingAcceleration = 2; // (Meters per Second Squared)

  // Max Path Following Turn Speeds
  public static final double maxThetaVelocity = 6.28; // (Radians per Second)
  public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)


  // Max speeds where its safe to X wheels
  public static final double maxSpeedToX = 0.25; // m/sec
  public static final double maxTurnToX = 20.0;  // degrees/sec


  // DataLogManager enabled
  public static final boolean dataLogging = false;

}
