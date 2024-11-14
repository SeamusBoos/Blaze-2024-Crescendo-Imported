package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
// General Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// Drive Imports
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;
// Shooter Imports
// Drive Imports
import frc.robot.Subsystems.RevMaxSwerve.Commands.SetFastMode;
import frc.robot.Subsystems.RevMaxSwerve.Commands.SetNormalMode;
import frc.robot.Subsystems.RevMaxSwerve.Commands.SetSlowMode;
import frc.robot.Subsystems.RevMaxSwerve.Commands.CrescendoCardinal.Smart90Lock;
import frc.robot.Subsystems.RevMaxSwerve.Commands.CrescendoCardinal.SmartPassAngle;
import frc.robot.Subsystems.RevMaxSwerve.Commands.CrescendoCardinal.SmartStageLock;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.LimitSwitches.LeftArmDownSwitch;
import frc.robot.Sensors.LimitSwitches.RightArmDownSwitch;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.Subsystems.RevMaxSwerve.Commands.AutoIntakeNote;
import frc.robot.Subsystems.RevMaxSwerve.Commands.FaceBackwards;
import frc.robot.Subsystems.RevMaxSwerve.Commands.FaceForward;
import frc.robot.Subsystems.RevMaxSwerve.Commands.FaceRight;
import frc.robot.Subsystems.RevMaxSwerve.Commands.FaceLeft;
// import frc.robot.SubmoduleSubsystemConstants.constsJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.OIConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.ClimbWheel;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Commands.GetRunSkippingPath;
import frc.robot.Commands.GetSkipPath;
import frc.robot.Commands.TunePitch;
import frc.robot.Commands.CarriageCommands.carriageBeltBackward;
import frc.robot.Commands.CarriageCommands.carriageBeltForward;
import frc.robot.Commands.CarriageCommands.carriageBeltOff;
import frc.robot.Commands.ClimberAndWheelCommands.climbDownLeft;
import frc.robot.Commands.ClimberAndWheelCommands.climbDownRight;
import frc.robot.Commands.ClimberAndWheelCommands.climbOff;
import frc.robot.Commands.ClimberAndWheelCommands.climbUpLeft;
import frc.robot.Commands.ClimberAndWheelCommands.climbUpRight;
import frc.robot.Commands.ClimberAndWheelCommands.climbWheelDown;
import frc.robot.Commands.ClimberAndWheelCommands.climbWheelOff;
import frc.robot.Commands.ClimberAndWheelCommands.climbWheelUp;
import frc.robot.Commands.ElevatorCommands.elevatorDownManual;
import frc.robot.Commands.ElevatorCommands.elevatorDownPID;
import frc.robot.Commands.ElevatorCommands.elevatorHoldPosition;
import frc.robot.Commands.ElevatorCommands.elevatorOff;
import frc.robot.Commands.ElevatorCommands.elevatorUpManual;
import frc.robot.Commands.ElevatorCommands.elevatorUpPID;
import frc.robot.Commands.IntakeCommands.fullIntakeOff;
import frc.robot.Commands.IntakeCommands.AutonomousNoteTrackingIntake;
import frc.robot.Commands.IntakeCommands.ODSIntake;
import frc.robot.Commands.IntakeCommands.RotatingNoteAlignment;
import frc.robot.Commands.IntakeCommands.StrafingNoteAlignment;
import frc.robot.Commands.IntakeCommands.intakeBackward;
import frc.robot.Commands.IntakeCommands.intakeForward;
import frc.robot.Commands.IntakeCommands.intakeManual;
import frc.robot.Commands.IntakeCommands.intakeOff;
import frc.robot.Commands.IntakeCommands.outtake;
import frc.robot.Commands.SensorAndLEDCommands.LEDsOrange;
import frc.robot.Commands.SensorAndLEDCommands.LEDsWhite;
import frc.robot.Commands.SensorAndLEDCommands.locateNoteInRobot;
import frc.robot.Commands.ShooterAndPivotCommands.AutoAlignOnly;
import frc.robot.Commands.ShooterAndPivotCommands.AutoPivotOnly;
import frc.robot.Commands.ShooterAndPivotCommands.AutoShoot;
import frc.robot.Commands.ShooterAndPivotCommands.DefaultPitch;
import frc.robot.Commands.ShooterAndPivotCommands.FullPassthrough;
import frc.robot.Commands.ShooterAndPivotCommands.FullPassthroughBloop;
import frc.robot.Commands.ShooterAndPivotCommands.PitchHoldPosition;
import frc.robot.Commands.ShooterAndPivotCommands.ShooterIdle;
import frc.robot.Commands.ShooterAndPivotCommands.SmartScore;
import frc.robot.Commands.ShooterAndPivotCommands.SpinUpWithPivot;
import frc.robot.Commands.ShooterAndPivotCommands.StayReady;
import frc.robot.Commands.ShooterAndPivotCommands.feederBackward;
import frc.robot.Commands.ShooterAndPivotCommands.feederForward;
import frc.robot.Commands.ShooterAndPivotCommands.feederOff;
import frc.robot.Commands.ShooterAndPivotCommands.pitchLowest;
import frc.robot.Commands.ShooterAndPivotCommands.pitchSubwoofer;
import frc.robot.Commands.ShooterAndPivotCommands.shoot;
import frc.robot.Commands.ShooterAndPivotCommands.Pass;
import frc.robot.Commands.ShooterAndPivotCommands.shooterFlywheelBackward;
import frc.robot.Commands.ShooterAndPivotCommands.shooterFlywheelForward;
import frc.robot.Commands.ShooterAndPivotCommands.shooterFlywheelOff;
import frc.robot.Commands.ShooterAndPivotCommands.shooterPitchDown;
import frc.robot.Commands.ShooterAndPivotCommands.shooterPitchOff;
import frc.robot.Commands.ShooterAndPivotCommands.shooterPitchUp;
import frc.robot.Commands.ShooterAndPivotCommands.spinUpFlywheels;
import frc.robot.Commands.ShooterAndPivotCommands.MiamiValleyPivot.PitchAmp;
import frc.robot.Commands.ShooterAndPivotCommands.MiamiValleyPivot.PitchCenter;
import frc.robot.Commands.ShooterAndPivotCommands.MiamiValleyPivot.PitchSource;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstProperties;
/**
 * Contains the robot definition, button bindings for teleop and autonomous configurations.
 */
public class RobotContainer {
  // define subsystems
  private final DriveSubsystemSwerve robotDrive = new DriveSubsystemSwerve();
  private final Intake intake = new Intake();
  private final CarriageBelt carriage = new CarriageBelt();
  private final Feeder feeder = new Feeder();
  private final ShooterFlywheels shooterFlywheels = new ShooterFlywheels();
  private final Climber climber = new Climber();
  private final ClimbWheel climbWheel = new ClimbWheel();
  private final Elevator elevator = new Elevator();
  private final CarriageODS carriageODS = new CarriageODS();
  private final IntakeODS intakeODS = new IntakeODS();
  private final LEDs leds = new LEDs();
  private final LeftArmDownSwitch leftDownSwitch = new LeftArmDownSwitch() ;
  private final RightArmDownSwitch rightDownSwitch = new RightArmDownSwitch();  

    private final ShooterPitch shooterPitch = new ShooterPitch();

  // private final ClimbWheelSubsystem climbWheel = new ClimbWheelSubsystem();
  private final ShooterLimelight shooterLL = new ShooterLimelight();
  private final IntakeLimelight intakeLL = new IntakeLimelight();
  // Joystick Controller (I/O)
  CommandXboxController driverController = new CommandXboxController(ConstJoysticks.kDriverControllerPort);
  CommandXboxController gunnerController = new CommandXboxController(ConstJoysticks.kGunnerControllerPort);
  // XboxController testingController = new XboxController(2);

  // Autonomous Chooser
  private ShuffleboardTab sbCompTab = Shuffleboard.getTab(ConstProperties.CompDashboard.COMPDASHNAME_STRING);
  SendableChooser<Command> autoChooserPathPlan = new SendableChooser<>();

  // Backup Cardinal Directions
  private Command pointF = Commands.run(() -> robotDrive.rotateToAngle(0,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointL = Commands.run(() -> robotDrive.rotateToAngle(90,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointB = Commands.run(() -> robotDrive.rotateToAngle(180,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  private Command pointR = Commands.run(() -> robotDrive.rotateToAngle(270,
  -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband), 
  -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)), robotDrive);
  
  private RepeatCommand repeatPointF = new RepeatCommand(pointF);
  private RepeatCommand repeatPointL = new RepeatCommand(pointL);
  private RepeatCommand repeatPointB = new RepeatCommand(pointB);
  private RepeatCommand repeatPointR = new RepeatCommand(pointR);

  /**
   * Constructor for RobotContainer class.
   */
  public RobotContainer() {
    //Register named autonomous commands
    NamedCommands.registerCommand("SpinUpShooter", new spinUpFlywheels(shooterFlywheels));
    NamedCommands.registerCommand("SpinUpTimeout", new spinUpFlywheels(shooterFlywheels).withTimeout(0.4));
    NamedCommands.registerCommand("Shoot", new shoot(shooterFlywheels, feeder, carriage).withTimeout(2));
    NamedCommands.registerCommand("ShootQuick", new shoot(shooterFlywheels, feeder, carriage).withTimeout(.25));

    NamedCommands.registerCommand("AutoShoot", new SmartScore(shooterPitch, shooterFlywheels, feeder, carriage, intake, shooterLL, robotDrive, elevator));
    NamedCommands.registerCommand("AutoShootTimeout", new SmartScore(shooterPitch, shooterFlywheels, feeder, carriage, intake, shooterLL, robotDrive, elevator).withTimeout(0.75));

    NamedCommands.registerCommand("SpinUpWithPivot", new SpinUpWithPivot(shooterPitch, shooterFlywheels, shooterLL));

    NamedCommands.registerCommand("FullPassthrough", new FullPassthrough(shooterFlywheels, feeder, carriage, intake));
    NamedCommands.registerCommand("FullPassthroughTimeout", new FullPassthrough(shooterFlywheels, feeder, carriage, intake).withTimeout(0.5));

    NamedCommands.registerCommand("FullPassthroughBloop", new FullPassthroughBloop(shooterFlywheels, shooterPitch, feeder, carriage, intake, intakeLL, shooterLL));
    NamedCommands.registerCommand("FullPassthroughBloopTimeout", new FullPassthroughBloop(shooterFlywheels, shooterPitch, feeder, carriage, intake, intakeLL, shooterLL).withTimeout(0.25));


    NamedCommands.registerCommand("FullLimelightIntake", new AutonomousNoteTrackingIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, robotDrive).withTimeout(3));

    NamedCommands.registerCommand("IntakeOff", new fullIntakeOff(intake, carriage).withTimeout(0.0000000001));
    NamedCommands.registerCommand("Intake", new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator));
    NamedCommands.registerCommand("LongIntake", new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator).withTimeout(2));
    NamedCommands.registerCommand("IntakeNoTimeout", new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator));
    NamedCommands.registerCommand("ManualIntake", new intakeManual(intake, carriage));

    NamedCommands.registerCommand("PitchSubwoofer", new pitchSubwoofer(shooterPitch).withTimeout(2)); 
    NamedCommands.registerCommand("pitchLowest", new pitchLowest(shooterPitch).withTimeout(1));
    NamedCommands.registerCommand("Pitch40", new PitchCenter(shooterPitch));
    NamedCommands.registerCommand("Pitch44", new PitchSource(shooterPitch));
    NamedCommands.registerCommand("PitchAmp", new PitchAmp(shooterPitch));
    NamedCommands.registerCommand("AutoShootPitchOnly", new AutoPivotOnly(shooterPitch, shooterFlywheels, feeder, carriage, shooterLL));
    NamedCommands.registerCommand("AutoShootPitchOnlyTimeout", new AutoPivotOnly(shooterPitch, shooterFlywheels, feeder, carriage, shooterLL).withTimeout(0.4));


    NamedCommands.registerCommand("GetSkipPath", new GetSkipPath());
    NamedCommands.registerCommand("RunSkippingPath", new GetRunSkippingPath());

    //Unused
    NamedCommands.registerCommand("CarriageBackwards", new carriageBeltBackward(carriage).withTimeout(0.35)); //originally 0.5
    NamedCommands.registerCommand("BloopShot", new Pass(shooterFlywheels, shooterPitch, feeder, carriage));





    // TELEOP Setup
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.

    autoChooserPathPlan = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    sbCompTab.add("Choose Path Planner Auto", autoChooserPathPlan).withSize(2, 1).withPosition(0, 0);

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
              -MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
              -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband),
              DriveConstants.kFieldCentric, true, DriveConstants.rotPt),
            robotDrive));

    carriageODS.setDefaultCommand(new locateNoteInRobot(intakeODS, carriageODS, shooterLL, intakeLL));

    shooterPitch.setDefaultCommand(new DefaultPitch(shooterPitch));

    shooterFlywheels.setDefaultCommand(new ShooterIdle(shooterFlywheels));

    // shooterPitch.setDefaultCommand(new TunePitch(shooterPitch));
    }

  private void configureBindings() {

    // ***TESTING BUTTONS*** - KEEP COMMENTED UNLESS IN USE
    // driverController.pov(0).whileTrue(new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator));
    // driverController.y().whileTrue(new shoot(shooterFlywheels, feeder, carriage));

    // ***DRIVER CONTROLS***
    // Reset heading
    driverController.back().whileTrue(new RunCommand(() -> robotDrive.zeroHeading()));
    // Set drivetrain into brakeing configuration
    driverController.start().whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));
    // Cardinal Direction Buttons
    // driverController.y().whileTrue(new FaceForward(robotDrive));
    driverController.y().whileTrue(new RotatingNoteAlignment(intakeLL, robotDrive));
    driverController.x().whileTrue(new Smart90Lock(robotDrive));
    driverController.b().whileTrue(new SmartPassAngle(robotDrive));
    driverController.a().whileTrue(new SmartStageLock(robotDrive)); 
    // Set speed modes
    driverController.leftBumper().onTrue(new SetSlowMode(robotDrive));
    driverController.rightBumper().onFalse(new SetNormalMode(robotDrive));
    driverController.rightBumper().onTrue(new SetFastMode(robotDrive));
    driverController.leftBumper().onFalse(new SetNormalMode(robotDrive));
    // Field Centric vs. Robot Centric
    // driverController.pov(90).toggleOnTrue(new RunCommand(() -> robotDrive.setFieldcentric(false)));
    // Scoring and Passing
    driverController.leftTrigger().whileTrue(new SmartScore(shooterPitch, shooterFlywheels, feeder, carriage, intake, shooterLL, robotDrive, elevator).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    driverController.rightTrigger().whileTrue(new Pass(shooterFlywheels, shooterPitch, feeder, carriage));
    driverController.pov(270).toggleOnTrue(new StayReady());

    // ***GUNNER CONTROLS***
     // full intake & outtake
    gunnerController.rightBumper().whileTrue(new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator));
    gunnerController.rightBumper().whileFalse(new fullIntakeOff(intake, carriage));
    gunnerController.pov(90).whileTrue(new intakeManual(intake, carriage));
    gunnerController.leftBumper().whileTrue(new outtake(intake, carriage));
    // // shooter & carriage
    gunnerController.leftTrigger().whileTrue(new AutoAlignOnly(shooterPitch, shooterFlywheels, shooterLL, robotDrive));
    gunnerController.rightTrigger().whileTrue(new spinUpFlywheels(shooterFlywheels));
    gunnerController.b().whileTrue(new shoot(shooterFlywheels, feeder, carriage));
    gunnerController.start().onTrue(new pitchSubwoofer(shooterPitch));
    gunnerController.back().onTrue(new pitchLowest(shooterPitch));
    // // climber
    gunnerController.pov(180).whileTrue(new climbUpRight(climber, rightDownSwitch)); //Need to reverse names. Right = Left
    gunnerController.pov(0).whileTrue(new climbDownRight(climber)); //And Left = Right Currently
    gunnerController.a().whileTrue(new climbUpLeft(climber, leftDownSwitch));
    gunnerController.y().whileTrue(new climbDownLeft(climber));
    // // elevator
    gunnerController.leftStick().whileTrue(new elevatorUpManual(elevator, intakeLL));
    gunnerController.leftStick().whileFalse(new elevatorHoldPosition(elevator, intakeLL));
    gunnerController.rightStick().whileTrue(new elevatorDownManual(elevator, intakeLL));
    gunnerController.rightStick().whileFalse(new elevatorHoldPosition(elevator, intakeLL));

    // // Stay Ready
    gunnerController.pov(270).toggleOnTrue(new StayReady());


    // CHAMPS
    // // full intake & outtake
    // new JoystickButton(gunnerController, Button.kRightBumper.value).whileTrue(new ODSIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, elevator));
    // new JoystickButton(gunnerController, Button.kRightBumper.value).whileFalse(new fullIntakeOff(intake, carriage));
    // new POVButton(gunnerController, 90).whileTrue(new intakeManual(intake, carriage));
    // new JoystickButton(gunnerController, Button.kLeftBumper.value).whileTrue(new outtake(intake, carriage));
    // // // shooter & carriage
    // new JoystickButton(gunnerController, Button.kX.value).whileTrue(new AutoShoot(shooterPitch, shooterFlywheels, feeder, carriage, shooterLL, robotDrive));
    // new POVButton(gunnerController, 270).whileTrue(new spinUpFlywheels(shooterFlywheels));
    // new JoystickButton(gunnerController, Button.kB.value).whileTrue(new shoot(shooterFlywheels, feeder, carriage));
    // new JoystickButton(gunnerController, Button.kStart.value).onTrue(new pitchSubwoofer(shooterPitch));
    // new JoystickButton(gunnerController, Button.kBack.value).onTrue(new pitchLowest(shooterPitch));
    // // // climber
    // new POVButton(gunnerController, 180).whileTrue(new climbUpRight(climber, rightDownSwitch)); //Need to reverse names. Right = Left
    // new POVButton(gunnerController, 0).whileTrue(new climbDownRight(climber)); //And Left = Right Currently
    // new JoystickButton(gunnerController, Button.kA.value).whileTrue(new climbUpLeft(climber, leftDownSwitch));
    // new JoystickButton(gunnerController, Button.kY.value).whileTrue(new climbDownLeft(climber));
    // // // elevator
    // new JoystickButton(gunnerController, Button.kLeftStick.value).whileTrue(new elevatorUpManual(elevator, intakeLL));
    // new JoystickButton(gunnerController, Button.kLeftStick.value).whileFalse(new elevatorHoldPosition(elevator, intakeLL));
    // new JoystickButton(gunnerController, Button.kRightStick.value).whileTrue(new elevatorDownManual(elevator, intakeLL));
    // new JoystickButton(gunnerController, Button.kRightStick.value).whileFalse(new elevatorHoldPosition(elevator, intakeLL));

    // NOT USED AT CHAMPS
    // new JoystickButton(gunnerController, Button.kStart.value).whileTrue(new FeedAndShoot(shooter, feeder));
    // new JoystickButton(gunnerController, Button.kStart.value).onFalse(new ShooterOff(shooter));

    // new JoystickButton(gunnerController, Button.kA.value).whileTrue(new FeedForward(feeder));
    // new JoystickButton(gunnerController, Button.kB.value).whileTrue(new AutoIntakeNote(robotDrive, intakeLL, intake));

    // new JoystickButton(gunnerController, Button.kX.value).onTrue(new RunCommand(() -> intakeLL.setPipeline(0), intakeLL));
    // new JoystickButton(gunnerController, Button.kY.value).onTrue(new RunCommand(() -> intakeLL.setPipeline(1), intakeLL));
    
    // *** TESTING CONTROLS ***
    // shooter pitch 
    // new JoystickButton(testingController, Button.kBack.value).whileTrue(new shooterPitchUp(shooterPitch));
    // new JoystickButton(testingController, Button.kBack.value).whileFalse(new shooterPitchOff(shooterPitch));
    // new JoystickButton(testingController, Button.kBack.value).whileFalse(new PitchHoldPosition(shooterPitch));

    // new JoystickButton(testingController, Button.kStart.value).whileTrue(new shooterPitchDown(shooterPitch));
    // new JoystickButton(testingController, Button.kStart.value).whileFalse(new shooterPitchOff(shooterPitch));
    // new JoystickButton(testingController, Button.kStart.value).whileFalse(new PitchHoldPosition(shooterPitch));

    // new JoystickButton(testingController, Button.kX.value).whileTrue(new shoot(shooterFlywheels, feeder, carriage));
    // new JoystickButton(testingController, Button.kRightBumper.value).whileTrue(new StrafingLLIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, robotDrive));
    // new JoystickButton(testingController, Button.kY.value).whileTrue(new AutoShoot(shooterPitch, shooterFlywheels, feeder, carriage, shooterLL, robotDrive));
    // new JoystickButton(testingController, Button.kLeftBumper.value).whileTrue(new RotatingLLIntake(intake, carriage, intakeODS, carriageODS, shooterLL, intakeLL, robotDrive));
  }

  /**
   * Retrieve the autonomous command that is selected in the Autonomous Chooser.
   *
   * @return Command
   */
  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return autoChooserPathPlan.getSelected();
  }
  void smartDashboardOut(){
    SmartDashboard.putNumber("getRightTriggerAxis", driverController.getRightTriggerAxis());
    SmartDashboard.putNumber("raw Driver RT", driverController.getRawAxis(2)); 
  }
}
