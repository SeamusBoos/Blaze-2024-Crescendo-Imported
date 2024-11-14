package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.Subsystems.Elevator;

public class elevatorDownManual extends Command {
    private final Elevator elevator;
    private final IntakeLimelight intakeLL;

    public elevatorDownManual(Elevator elevator, IntakeLimelight intakeLL){
        this.elevator = elevator;
        this.intakeLL = intakeLL;
        addRequirements(elevator); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(elevator.getPosition()<ConstElevator.elevatorSlowZoneDown && elevator.getPosition()>ConstElevator.loweredPos){
            elevator.setSpeed(-ConstElevator.elevatorFinishSpeed);
            elevator.run();
        } else if(elevator.getPosition()>ConstElevator.loweredPos){
            elevator.setSpeed(-ConstElevator.elevatorFallSpeed);
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