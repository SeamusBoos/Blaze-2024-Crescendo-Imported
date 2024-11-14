package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class intakeOff extends Command {
    private final Intake intake;

    public intakeOff(Intake subsystem){
        intake = subsystem;
        addRequirements(subsystem); 
    }
    
    public void initialize() {
        intake.setSpeed(0);
    }

    @Override
    public void execute() {
        intake.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}