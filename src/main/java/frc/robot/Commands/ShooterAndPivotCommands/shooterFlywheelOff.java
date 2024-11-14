package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterFlywheels;

public class shooterFlywheelOff extends Command {
    private final ShooterFlywheels shooterFlywheels;

    public shooterFlywheelOff(ShooterFlywheels subsystem){
        shooterFlywheels = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterFlywheels.setSpeed(0);
    }

    @Override
    public void execute() {
        shooterFlywheels.run();
    }

    @Override
    public void end(boolean interrupted) {
        shooterFlywheels.stop();
    }
}