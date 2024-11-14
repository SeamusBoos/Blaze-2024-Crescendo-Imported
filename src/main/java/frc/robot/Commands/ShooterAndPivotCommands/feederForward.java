package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder;

public class feederForward extends Command {
    private final Feeder feeder;

    public feederForward(Feeder subsystem){
        feeder = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        feeder.setSpeed(0.5);
    }

    @Override
    public void execute() {
        feeder.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
