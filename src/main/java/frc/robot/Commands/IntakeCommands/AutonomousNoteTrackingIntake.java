package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstAuto;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.GamePieceAlignmentConstants;

/**
 * Command to reorient the robot to face forward.
 */
public class AutonomousNoteTrackingIntake extends Command {
  private final Intake intake;
  private final CarriageBelt carriageBelt;
  private final IntakeODS intakeODS;
  private final CarriageODS carriageODS;
  private final ShooterLimelight shooterLL;
  private final IntakeLimelight intakeLL;
  private final DriveSubsystemSwerve robotDrive;
  private final Timer timer = new Timer();
  private final Timer endTimer = new Timer();
  private double XPower;
  private double YPower;
  private PIDController XController;
  private PIDController YController;
  private double xTarget = 0;
  private double yTarget = -30;
  private double NoteX;
  private double NoteY;
  private double xTolerance = 10;
  private double xError;
  private String noteLocation = "No Note";
  boolean notePassedIntake = false;

  /**
   * Add subsystem and configure PID controller.
   *
   * @param subsystem drive subsystem
   */
  public AutonomousNoteTrackingIntake(Intake intakeSS, CarriageBelt carriageSS, IntakeODS iODS, CarriageODS cODS, ShooterLimelight LLShooter, IntakeLimelight LLIntake, DriveSubsystemSwerve driveSS) {
    intake = intakeSS;
    carriageBelt = carriageSS;
    intakeODS = iODS;
    carriageODS = cODS;
    shooterLL = LLShooter;
    intakeLL = LLIntake;
    robotDrive = driveSS;
    addRequirements(intakeSS, carriageSS, iODS, cODS, LLShooter, LLIntake, driveSS);
    XController = new PIDController(GamePieceAlignmentConstants.P, GamePieceAlignmentConstants.I, GamePieceAlignmentConstants.D);
    YController = new PIDController(GamePieceAlignmentConstants.P, GamePieceAlignmentConstants.I, GamePieceAlignmentConstants.D);
  }

  @Override
  public void initialize() {
    ConstAuto.skipNextPath = false;
    notePassedIntake = false;
    timer.reset();
    endTimer.reset();
    intake.setSpeed(0.5);
    carriageBelt.setSpeed(0.5);
    XController.setSetpoint(xTarget);
    YController.setSetpoint(yTarget);

    if(intakeLL.getV()!=1){
      ConstAuto.skipNextPath = true;
      end(true);
    }
  }

  @Override
  public void execute() {
    NoteX = intakeLL.getX();
    NoteY = intakeLL.getY();

    XPower = XController.calculate(NoteX);
    YPower = YController.calculate(NoteY);

    xError = xTarget - NoteX;

    if(intakeLL.getV()==1 && Math.abs(xError)<xTolerance){
    ConstAuto.skipNextPath = false;
    intake.runSpeed();
    carriageBelt.runSpeed();
    robotDrive.drive(XPower, YPower, 0, false, true);
    } else if(intakeLL.getV()==1){
    ConstAuto.skipNextPath = false;
    robotDrive.drive(XPower, 0, 0, false, true);
    }
    else {
      endTimer.start();
    }

    if((intakeODS.getSightStatus()&&carriageODS.getSightStatus())||carriageODS.getSightStatus()){
        carriageODS.setHasNote(true);
        carriageODS.setNoteLocation("Carriage");
        noteLocation = carriageODS.getNoteLocation();
    } else if (intakeODS.getSightStatus()){
        notePassedIntake = true;
        carriageODS.setHasNote(true);
        carriageODS.setNoteLocation("Intake");
        noteLocation = carriageODS.getNoteLocation();
    } else {
        carriageODS.setHasNote(false);
        carriageODS.setNoteLocation("No Note");
        noteLocation = carriageODS.getNoteLocation();
    }

    if (noteLocation.equals("Carriage") && notePassedIntake){
    intake.stop();
    carriageBelt.setSpeed(-0.5);
    timer.start();
    }
    else if (noteLocation.equals("Intake")){
    }
    else {
    intake.runSpeed();
    carriageBelt.setSpeed(0.5);
    carriageBelt.runSpeed();
    }

    if(timer.get()>0.25){
      timer.stop();
      carriageBelt.stop();
    }

    if(endTimer.get()>0.75){
      endTimer.stop();
      endTimer.reset();
      intake.stop();
      carriageBelt.stop();
      end(true);
    }

    if(carriageBelt.getVelocity()>100||carriageBelt.getVelocity()<-100){
      shooterLL.setLedMode(3);
      intakeLL.setLedMode(3);
  } else {
      shooterLL.setLedMode(1);
      intakeLL.setLedMode(1);
  }

  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    carriageBelt.stop();
  }
}
