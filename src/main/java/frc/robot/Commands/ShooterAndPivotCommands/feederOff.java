package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Feeder;

public class feederOff extends Command {
    private final Feeder feeder;

    public feederOff(Feeder subsystem){
        feeder = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        feeder.setSpeed(0);
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
