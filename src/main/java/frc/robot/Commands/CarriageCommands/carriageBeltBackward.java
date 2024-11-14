package frc.robot.Commands.CarriageCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CarriageBelt;

public class carriageBeltBackward extends Command {
    private final CarriageBelt carriageBelt;

    public carriageBeltBackward(CarriageBelt subsystem){
        carriageBelt = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        carriageBelt.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        carriageBelt.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        carriageBelt.stop();
    }
}
