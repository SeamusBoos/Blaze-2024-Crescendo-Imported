package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;
import frc.robot.Subsystems.Elevator;

public class elevatorHoldPosition extends Command {
    private final Elevator elevator;
    private final IntakeLimelight intakeLL;
    private double holdPos;

    public elevatorHoldPosition(Elevator subsystem, IntakeLimelight LLintake){
        elevator = subsystem;
        intakeLL = LLintake;
        addRequirements(subsystem); 
    }

    public void initialize() {
        if (elevator.getPosition()>ConstElevator.raisedPos){
            holdPos = ConstElevator.raisedPos;
        }
        else if(elevator.getPosition()<ConstElevator.loweredPos){
            holdPos = ConstElevator.loweredPos;
        } else{
        holdPos = elevator.getPosition();}
    }

    @Override
    public void execute() {
        elevator.seekPosition(holdPos);
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