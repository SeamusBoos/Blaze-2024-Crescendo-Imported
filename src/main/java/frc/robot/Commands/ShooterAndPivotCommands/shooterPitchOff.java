package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPitch;

public class shooterPitchOff extends Command {
    private final ShooterPitch shooterPitch;

    public shooterPitchOff(ShooterPitch subsystem){
        shooterPitch = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterPitch.setSpeed(0);
    }

    @Override
    public void execute() {
        shooterPitch.run();
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
