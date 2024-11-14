package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbOff extends Command {
    private final Climber climber;

    public climbOff(Climber subsystem){
        climber = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        climber.setSpeed(0);
    }

    @Override
    public void execute() {
        climber.run();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}