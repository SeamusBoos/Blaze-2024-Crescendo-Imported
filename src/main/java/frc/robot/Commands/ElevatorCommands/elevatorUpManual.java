package frc.robot.Commands.ElevatorCommands;

import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class elevatorUpManual extends Command {
    private final Elevator elevator;
    private final IntakeLimelight intakeLL;

    public elevatorUpManual(Elevator elevator, IntakeLimelight intakeLL){
        this.elevator = elevator;
        this.intakeLL = intakeLL;
        addRequirements(elevator); 
    }

    public void initialize() {
        ConstShooter.NoteInRobot = false;
    }

    @Override
    public void execute() {
        if(elevator.getPosition()>ConstElevator.elevatorSlowZoneUp && elevator.getPosition()<ConstElevator.raisedPos){
            elevator.setSpeed(ConstElevator.elevatorFinishSpeed);
        elevator.run();
        } else if(elevator.getPosition()<ConstElevator.raisedPos){
            elevator.setSpeed(ConstElevator.elevatorRiseSpeed);
        elevator.run();
        } else {
            elevator.stop();
        }

        if(elevator.getPosition()>30){
            intakeLL.setPipeline(1);
        } else {
            intakeLL.setPipeline(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}