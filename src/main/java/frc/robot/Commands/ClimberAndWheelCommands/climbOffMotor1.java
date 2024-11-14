package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbOffMotor1 extends Command {
    private final Climber climber;

    public climbOffMotor1(Climber subsystem){
        climber = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        climber.setSpeed(0);
    }

    @Override
    public void execute() {
        climber.runMotor1();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}