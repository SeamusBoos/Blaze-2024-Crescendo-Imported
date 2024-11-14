package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbWheel;

public class climbWheelDown extends Command {
    private final ClimbWheel wheel;

    public climbWheelDown(ClimbWheel subsystem){
        wheel = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        wheel.setSpeed(-1);
    }

    @Override
    public void execute() {
        wheel.setSpeed(-1);
        wheel.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        wheel.stop();
    }
}
