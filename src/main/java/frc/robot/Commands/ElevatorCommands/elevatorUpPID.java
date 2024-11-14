package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.Subsystems.Elevator;

public class elevatorUpPID extends Command {
    private final Elevator elevator;
    private final IntakeLimelight intakeLL;

    public elevatorUpPID(Elevator subsystem, IntakeLimelight intakeLL){
        elevator = subsystem;
        this.intakeLL = intakeLL;
        addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.seekPosition(75);

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