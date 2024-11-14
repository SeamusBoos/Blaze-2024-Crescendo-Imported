package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Sensors.ODS.CarriageODS;
import frc.robot.Sensors.ODS.IntakeODS;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

public class ODSIntake extends Command {

    String noteLocation = "No Note";
    boolean notePassedIntake = false;
    boolean noteDetectedAtCarriage = false;

    private final Intake intake;
    private final CarriageBelt carriageBelt;
    private final IntakeODS intakeODS;
    private final CarriageODS carriageODS;
    private final ShooterLimelight shooterLL;
    private final IntakeLimelight intakeLL;
    private final Elevator elevator;

    private final Timer timer = new Timer();

    public ODSIntake(Intake intakeSS, CarriageBelt carriageSS, IntakeODS iODS, CarriageODS cODS, ShooterLimelight LLShooter, IntakeLimelight LLIntake, Elevator elevatorSS){
        intake = intakeSS;
        carriageBelt = carriageSS;
        intakeODS = iODS;
        carriageODS = cODS;
        shooterLL = LLShooter;
        intakeLL = LLIntake;
        elevator = elevatorSS;
        addRequirements(intake, carriageBelt, intakeODS, carriageODS); 
    }

    public void initialize() {
        notePassedIntake = false;
        noteDetectedAtCarriage = false;
        intake.setSpeed(0.5);
        carriageBelt.setSpeed(0.5);
        timer.reset();
        ConstShooter.NoteInRobot = false;
    }

    @Override
    public void execute() {
        intake.runSpeed();
        carriageBelt.runSpeed();
        if((intakeODS.getSightStatus()&&carriageODS.getSightStatus())||carriageODS.getSightStatus()){
            carriageODS.setHasNote(true);
            carriageODS.setNoteLocation("Carriage");
            noteDetectedAtCarriage = true;
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
        // shooterLL.setLedMode(2);
        // intakeLL.setLedMode(2);
        intake.stop();
        carriageBelt.setSpeed(-0.5);
        timer.restart();
        }
        else if (noteLocation.equals("Intake")){
        // shooterLL.setLedMode(3);
        // intakeLL.setLedMode(3);
        }
        else {
        // shooterLL.setLedMode(1);
        // intakeLL.setLedMode(1);
        carriageBelt.runSpeed();
        intake.runSpeed();
        }

        if(timer.get()>0.25 && noteDetectedAtCarriage){
        // timer.stop();
//        DriverStation.reportWarning("timer stop carriage", true);
        carriageBelt.stop();
        intake.stop();
        }

        if(timer.get()<0.5 && noteDetectedAtCarriage){
        // shooterLL.setLedMode(2);
        // intakeLL.setLedMode(2);
        }
        else{
        // shooterLL.setLedMode(1);
        // intakeLL.setLedMode(1);
        timer.stop();
        }

        if(elevator.getPosition()>30){
        intake.stop();
        carriageBelt.stop();
        }

        if(carriageBelt.getVelocity()>100||carriageBelt.getVelocity()<-100){
            shooterLL.setLedMode(3);
            intakeLL.setLedMode(3);
        } else {
            shooterLL.setLedMode(1);
            intakeLL.setLedMode(1);
        }

        if(intakeODS.getSightStatus() || carriageODS.getSightStatus()) {
            ConstShooter.NoteInRobot = true;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        carriageBelt.stop();
        shooterLL.setLedMode(1);
        intakeLL.setLedMode(1);
    }
}
