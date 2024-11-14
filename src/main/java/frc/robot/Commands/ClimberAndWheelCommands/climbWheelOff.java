package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbWheel;

public class climbWheelOff extends Command {
    private final ClimbWheel intake;

    public climbWheelOff(ClimbWheel subsystem){
        intake = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        intake.setSpeed(0);
    }

    @Override
    public void execute() {
        intake.setSpeed(0);
        intake.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
