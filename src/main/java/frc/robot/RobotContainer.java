// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.lib.Ranging.CargoBallDataPoint;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.*;
import frc.robot.commands.Autonomous.FiveBallAuto;
import frc.robot.commands.Autonomous.ThreeBallAuto;
import frc.robot.commands.Autonomous.TwoBallAuto;
import frc.robot.commands.Deprecated.SetSwerveAngle;
import frc.lib.oi.controller.TriggerButton;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.DejamBallPath;
import frc.robot.commands.groups.PanicIntake;
import frc.robot.commands.groups.StopAutoIntake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  private SendableChooser<Command> autoChooser;

  // The robot's subsystems
  // public final Intake mIntake;
  public final Drivetrain mDrivetrain;

  public final Turret mTurret;
  public final ShooterHood mShooterHood;
  public final Shooter mShooter;

  public final Intake mIntake;
  public final CargoFunnel mCargoFunnel;

  public final BottomLift mBottomLift;
  public final IntakeDeploy mIntakeDeploy;
  public final Climb mClimb;

  // Joysticks
  public CommandXboxController controller0;
  public CommandXboxController controller1;

  //Cameras
  public UsbCamera intakeCamera;
  public MjpegServer virtualCamera;

  // public final LimeLight limeLightCamera;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public CargoBallInterpolator cargoBallInterpolator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    initInterpolator();

    // mIntake = new Intake();
    mDrivetrain = new Drivetrain();
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain));
         
    mClimb = new Climb();
    mClimb.setDefaultCommand(new ClimbSticks(mClimb, mDrivetrain));

    mTurret = new Turret(mDrivetrain);
    mTurret.setDefaultCommand(new TurretSticks(mTurret, mClimb));
  
    mShooterHood = new ShooterHood();
    mShooterHood.setDefaultCommand(new HoldHoodAngle(mShooterHood, mTurret, cargoBallInterpolator ));
    
    mShooter = new Shooter();
    // mShooter.setDefaultCommand(new DefaultShooter(mShooter, mTurret,
    // cargoBallInterpolator));
    
    mBottomLift = new BottomLift();
    mBottomLift.setDefaultCommand(new DefaultBottomLift(mBottomLift));

    mCargoFunnel = new CargoFunnel(mBottomLift);
    mCargoFunnel.setDefaultCommand(new DefaultCargoFunnel(mCargoFunnel));
    
    mIntake = new Intake();
    mIntake.setDefaultCommand(new DefaultIntake(mIntake));

    mIntakeDeploy = new IntakeDeploy();
    mIntakeDeploy.setDefaultCommand(new DefaultIntakeDeploy(mIntakeDeploy));
   
    controller0 = new CommandXboxController(0);
    controller1 = new CommandXboxController(1);

    setupAutoSelector();

    // limeLightCamera = new LimeLight();

    // Smartdashboard Subsystems
    SmartDashboard.putData(mShooter);
    SmartDashboard.putData(mShooterHood);
    SmartDashboard.putData(mTurret);
    SmartDashboard.putData(mIntake);
    SmartDashboard.putData(mCargoFunnel);
    SmartDashboard.putData(mDrivetrain);
    SmartDashboard.putData(mBottomLift);
    SmartDashboard.putData(mIntakeDeploy);
    SmartDashboard.putData(mClimb);

    // SmartDashboard Buttons
    // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // SmartDashboard.putData("Auto Mode", m_chooser);

    //Initialize cameras
    initCamera();
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    SmartDashboard.putData("Increase Main Shooter Speed", new ChangeMainShooterSpeed(mShooter, 50));
    SmartDashboard.putData("Decrease Main Shooter Speed", new ChangeMainShooterSpeed(mShooter, -50));

    SmartDashboard.putData("Increase Second Shooter Speed", new ChangeSecondaryShooterSpeed(mShooter, 50));
    SmartDashboard.putData("Decrease Second Shooter Speed", new ChangeSecondaryShooterSpeed(mShooter, -50));

    if (Constants.isSingleController) {
      // Triggers
      controller0.leftTrigger(0.4).whileTrue(new AutoTurretAim(mTurret, mClimb));
      controller0.leftTrigger(0.4).whileTrue(new AutoLimelightHood(mTurret, mShooterHood, cargoBallInterpolator));
      controller0.leftTrigger(0.4).whileTrue(new AutoLimelightMainShooter(mTurret, mShooter, cargoBallInterpolator));
      controller0.leftTrigger(0.4).whileTrue(new AutoLimelightSecondShooter(mTurret, mShooter, cargoBallInterpolator));

      controller0.rightTrigger(0.4).whileTrue(new AutoShoot(mCargoFunnel, mBottomLift,
          mShooter, mShooterHood, mTurret, mDrivetrain));

      // Bumpers
      controller0.leftBumper().whileTrue(new DejamBallPath(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy));
      controller0.rightBumper().onTrue(new MoveTurretToAngle(mTurret, 0));

      // POV
      controller0.povUp().whileTrue(new MoveHood(mShooterHood, 0.25));
      controller0.povDown().whileTrue(new MoveHood(mShooterHood, -0.25));

      controller0.povLeft().whileTrue(new MoveTurret(mTurret, 0.3));
      controller0.povRight().whileTrue(new MoveTurret(mTurret, -0.3));

      // ABXY
      controller0.a().onTrue(new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, false));

      controller0.b().onTrue(new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy));

      controller0.x().onTrue(new ChangeMainShooterSpeed(mShooter, -100)
          .andThen(new ChangeSecondaryShooterSpeed(mShooter, -100))
          .andThen(new StartShooter(mShooter)));
      controller0.y().onTrue(new ChangeMainShooterSpeed(mShooter, 100)
          .andThen(new ChangeSecondaryShooterSpeed(mShooter, 100))
          .andThen(new StartShooter(mShooter)));

      // Joysticks
      controller0.leftStick().onTrue(new StartShooter(mShooter));
      controller0.rightStick().onTrue(new StopShooter(mShooter));

      // Start/Back
      controller0.start().onTrue(new ResetGyro(mDrivetrain));

    } else {
  /*
  controller0 buttons
  */
    //-Triggers
    // TriggerButton autoAimButton = new TriggerButton(controller0, .4, 'l');
    // autoAimButton.whileActiveContinuous(new AutoTurretAim(mTurret, mClimb));
    // autoAimButton.whileActiveContinuous(new AutoLimelightHood(mTurret,
    // mShooterHood, cargoBallInterpolator));
    // autoAimButton.whileActiveContinuous(new AutoLimelightMainShooter(mTurret,
    // mShooter, cargoBallInterpolator));
    // autoAimButton.whileActiveContinuous(new AutoLimelightSecondShooter(mTurret,
    // mShooter, cargoBallInterpolator));
    controller0.leftTrigger(0.4).whileTrue(new AutoTurretAim(mTurret, mClimb));
    controller0.leftTrigger(0.4).whileTrue(new AutoLimelightHood(mTurret, mShooterHood, cargoBallInterpolator));
    controller0.leftTrigger(0.4).whileTrue(new AutoLimelightMainShooter(mTurret, mShooter, cargoBallInterpolator));
    controller0.leftTrigger(0.4).whileTrue(new AutoLimelightSecondShooter(mTurret, mShooter, cargoBallInterpolator));

    controller0.rightTrigger(0.4).whileTrue(new AutoShoot(mCargoFunnel, mBottomLift,
            mShooter, mShooterHood, mTurret, mDrivetrain));


      // JoystickButton panicIntakeButton1 = new JoystickButton(controller0, XboxController.Button.kRightBumper.value);
      // panicIntakeButton1.whileHeld(new PanicIntake(mIntake, mIntakeDeploy, mBottomLift, mCargoFunnel));

      controller0.rightBumper().onTrue(new SetSlowMode(mDrivetrain, true));
      controller0.rightBumper().onFalse(new SetSlowMode(mDrivetrain, false));

      // JoystickButton intakeOrientCameraButton = new JoystickButton(controller0, XboxController.Button.kLeftBumper.value);

    //-D-Pad
    // POVButton xPatternButtonUp = new POVButton(controller0, 0);
    // xPatternButtonUp.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45,
    // 45));
     
    // POVButton xPatternButtonDown = new POVButton(controller0, 180);
    // xPatternButtonDown.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45,
    // 45));
    // POVButton xPatternButtonLeft = new POVButton(controller0, 270);
    // xPatternButtonLeft.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45,
    // 45));

    // POVButton xPatternButtonRight = new POVButton(controller0, 90);
    // xPatternButtonRight.whenPressed(new HomeIntakeDeploy(mIntakeDeploy));
    //-ABXY
    controller0.povUp().onTrue(
      new ChangeMainShooterSpeed(mShooter, 50)
      .andThen(new ChangeSecondaryShooterSpeed(mShooter, 50))
      .andThen(new StartShooter(mShooter)));

    controller0.povDown().onTrue(
      new ChangeMainShooterSpeed(mShooter, -50)
      .andThen(new ChangeSecondaryShooterSpeed(mShooter, -50))
      .andThen(new StartShooter(mShooter)));

    // JoystickButton increaseSecondShooterSpeed = new JoystickButton(controller0,
    // XboxController.Button.kX.value);
    // increaseSecondShooterSpeed.whenPressed(new
    // ChangeSecondaryShooterSpeed(mShooter, 50));

    // JoystickButton decreaseSecondShooterSpeed = new JoystickButton(controller0,
    // XboxController.Button.kA.value);
    // decreaseSecondShooterSpeed.whenPressed(new
    // ChangeSecondaryShooterSpeed(mShooter, -50));

    //-Other Buttons
    controller0.start().onTrue(new ResetGyro(mDrivetrain));

    


  /*
  controller1 Buttons
  */
    //-Triggers and Bumpers
    controller0.leftTrigger(0.4).whileTrue(new DejamBallPath(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy));

    // JoystickButton driverBPanicButton = new JoystickButton(controller1,
    // XboxController.Button.kLeftBumper.value);
    // driverBPanicButton.whileHeld(new PanicIntake(mIntake, mIntakeDeploy,
    // mBottomLift, mCargoFunnel));

    // TriggerButton highGoalShotButton = new TriggerButton(controller1, .3, 'l');
    // highGoalShotButton.whileActiveContinuous(new NewHoodTarget(mShooterHood,
    // -152));
    // highGoalShotButton.whileActiveContinuous(new SetShooterSpeedTargets(mShooter,
    // 1750, 2650));
    // highGoalShotButton.whileActiveContinuous(new MoveTurretToAngle(mTurret,
    // 0.0));
      
    // JoystickButton autoTrackHub = new JoystickButton(controller1,
    // XboxController.Button.kRightBumper.value);
    // // lowGoalShotButton.whileActiveOnce(new NewHoodTarget(mShooterHood, 152),
    // true);
    // // lowGoalShotButton.whileActiveOnce(new SetShooterSpeedTargets(mShooter,
    // 1200, 0), true);
    // // lowGoalShotButton.whileActiveOnce(new MoveTurretToAngle(mTurret, 180));
      
    //-D-Pad
    controller0.povLeft().whileTrue(new MoveHood(mShooterHood, .25));

    controller0.povRight().whileTrue(new MoveHood(mShooterHood, -.25));

    // POVButton moveTurretLeftButton = new POVButton(controller1, 270);
    // moveTurretLeftButton.whileHeld(new MoveTurret(mTurret, -.30));

    // POVButton moveTurretRightButton = new POVButton(controller1, 90);
    // moveTurretRightButton.whileHeld(new MoveTurret(mTurret, .25));


    //-ABXY
    // controller0.x().onTrue(new InstantCommand(() -> mShooter.setMainShooterPower(0.65)));
    controller0.x().onTrue(new StartShooter(mShooter));

    controller0.a().onTrue(new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy, true));
      // autoIntakeButton.whenPressed(new ChangeIntakeState(mIntakeDeploy, true), true);

      controller0.b().onTrue(new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mIntakeDeploy));

      // controller0.y().onTrue(new InstantCommand(() -> mShooter.setMainShooterPower(0.0)));
      controller0.y().onTrue(new StopShooter(mShooter));


    
      //-Other Buttons
      
      // JoystickButton climbModeOffButton = new JoystickButton(controller1,
      // XboxController.Button.kBack.value);
      // climbModeOffButton.whenPressed(new ClimbModeOff(mClimb, mIntake,
      // mDrivetrain));

      // JoystickButton climbModeOnButton = new JoystickButton(controller1,
      // XboxController.Button.kStart.value);
      // climbModeOnButton.whenPressed(new ClimbModeOn(mClimb, mIntakeDeploy, mIntake,
      // mDrivetrain));
      // climbModeOnButton.whenPressed(new MoveTurretToAngle(mTurret, 180));

      // JoystickButton reverseIntakeButton = new JoystickButton(controller1, XboxController.Button.kRightStick.value);
      // reverseIntakeButton.whenPressed(new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy, false));

    }

  /*
  SmartDashBoard Buttons
  */
      // SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));
      // SmartDashboard.putData("90 Wheels", new SetSwerveAngle(mDrivetrain, 90, 90, 90, 90));
      // SmartDashboard.putData("180 Wheels", new SetSwerveAngle(mDrivetrain, 180, 180, 180, 180));
      // SmartDashboard.putData("270 Wheels", new SetSwerveAngle(mDrivetrain, 270, 270, 270, 270));


      // SmartDashboard.putData("Up 1 Angle", new NewHoodTarget(mShooterHood, Math.floor(mShooterHood.getHoodTarget() + 1)));
      // SmartDashboard.putData("Down 1 Angle", new NewHoodTarget(mShooterHood, Math.floor(mShooterHood.getHoodTarget() + -1)));

      // SmartDashboard.putData("Deploy Intake", new ChangeIntakeState(mIntakeDeploy, true));
      // SmartDashboard.putData("Retract Intake", new ChangeIntakeState(mIntakeDeploy, false));
      // SmartDashboard.putData("Reset Intake Encoder", new ResetIntakeDeployEncoder(mIntakeDeploy));

      SmartDashboard.putData("Home Intake", new HomeIntakeDeploy(mIntakeDeploy));

      SmartDashboard.putData("Turret 0", new MoveTurretToAngle(mTurret, 0));
      // SmartDashboard.putData("Turret 90", new MoveTurretToAngle(mTurret, 90));
      SmartDashboard.putData("Turret 180", new MoveTurretToAngle(mTurret, 180));
      
      SmartDashboard.putData("Reset Odometry", new ResetOdometry(mDrivetrain));

      // SmartDashboard.putData("0 Hood", new NewHoodTarget(mShooterHood, 0.0));
      // SmartDashboard.putData("120 Hood", new NewHoodTarget(mShooterHood, 120.0));
      // SmartDashboard.putData("-120 Hood", new NewHoodTarget(mShooterHood, -120.0));
      
    
    /*
    SmartDashboard.putData("0 Hood", new MoveHoodToAngle(mShooterHood, 0.0));
    SmartDashboard.putData("Top Hood", new MoveHoodToAngle(mShooterHood, 140.0));
    SmartDashboard.putData("Bottom Hood", new MoveHoodToAngle(mShooterHood, -140.0));
    // SmartDashboard.putData("100 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("120 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("130 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));

    SmartDashboard.putData("turret 180", new MoveTurretToAngle(mTurret, 180, 1));
    SmartDashboard.putData("turret 270", new MoveTurretToAngle(mTurret, 270, 1));
    SmartDashboard.putData("turret 90", new MoveTurretToAngle(mTurret, 90, 1));

    
*/
    // SmartDashboard.putData("turret 180", new MoveTurretToAngle(mTurret, 180));
    // SmartDashboard.putData("turret 270", new MoveTurretToAngle(mTurret, 270));
    // SmartDashboard.putData("turret 90", new MoveTurretToAngle(mTurret, 90));
  }

  private void setupAutoSelector() {
    // Auto Commands

    //Command driveStraightNoShootAuto = new AutoFollowPath(mDrivetrain, new StraightPath(2.0, 0).generateSwerveTrajectory(), true, false, 0.0);
    //Command testPathAuto = new AutoFollowPath(mDrivetrain, new TestPath(mDrivetrain, 90.0).generateSwerveTrajectory(), true, true, 90.0);
    //Command p3s1mAuto = new AutoP3S1M(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mTopLift, mBottomLift,
    //  mDrivetrain, mIntake, mIntakeDeploy);
    Command threeBallAuto = new ThreeBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    Command twoBallAuto = new TwoBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    Command fiveBallAuto = new FiveBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    autoChooser = new SendableChooser<>();


    autoChooser.setDefaultOption("Do Nothing", null);
    //autoChooser.addOption("Drive Straight (No Shoot)", driveStraightNoShootAuto);
    //autoChooser.addOption("Test Path", testPathAuto);
    //autoChooser.addOption("P3 Shoot1 Move", p3s1mAuto);
    autoChooser.addOption("3 Ball Auto", threeBallAuto);
    autoChooser.addOption("5 Ball Auto", fiveBallAuto);
    autoChooser.addOption("2 Ball Auto Straight", twoBallAuto);

    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  
  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public CommandXboxController getController0() {
    return controller0;
  }

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

  private void initCamera() {
    intakeCamera = CameraServer.startAutomaticCapture();
    //intakeCamera.setConnectionStrategy(ConnectionStrategy.kAutoManage);
    // intakeCamera.setFPS(20);
    //intakeCamera.setResolution(250, 120);
    

    //virtualCamera = CameraServer.addSwitchedCamera("Drive Camera");
    //virtualCamera.setSource(intakeCamera);
  }

  public void initInterpolator() {
    cargoBallInterpolator = new CargoBallInterpolator();

    // Example adding point
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(limeLightDistance, Main, Backspin, Hood, realDistance));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(37.7, 1750, 2650, -152, 24.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(47.6, 1700, 2550, -125, 48.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(63.7, 1836, 2720, -93.4, 48.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(71.6, 1900, 2800, -50.1, 72.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(95.7, 2050, 2800, 25.8, 96.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(119.4, 2200, 2850, 94.5, 120.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(141.5, 2600, 3100, 142.6, 144.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(165, 2800, 3250, 148.5, 168.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(177.4, 2850, 3300, 148.5, 168.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(188.9, 2750, 3450, 148.5, 192.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(213, 3050, 3500, 148.5, 216.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(236, 3250, 3500, 148.5, 240.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(265, 3300, 3850, 148.5, 264.0));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(280, 3450, 4700, 138, 288.0));
  }


  // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(37.7, 1750, 2650, -152, 24.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(47.6, 1700, 2550, -125, 48.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(71.6, 1900, 2800, -78.5, 72.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(95.7, 1800, 3050, 2, 96.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(119.4, 2200, 2850, 110, 120.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(141.5, 2450, 3000, 135.6, 144.0));//2350, 2850
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(165, 2600, 3250, 148.5, 168.0));//2600, 3100
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(188.9, 2750, 3450, 148.5, 192.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(213, 3050, 3500, 148.5, 216.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(236, 3250, 3500, 148.5, 240.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(265, 3300, 3850, 148.5, 264.0));
  //   cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(280, 3450, 4700, 138, 288.0));

} 
