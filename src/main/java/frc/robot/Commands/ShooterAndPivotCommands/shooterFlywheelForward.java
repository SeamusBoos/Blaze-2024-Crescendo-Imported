package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterFlywheels;

public class shooterFlywheelForward extends Command {
    private final ShooterFlywheels shooterFlywheels;

    public shooterFlywheelForward(ShooterFlywheels subsystem){
        shooterFlywheels = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterFlywheels.setTargetVelocity(ConstShooter.defVelocity);
    }

    @Override
    public void execute() {
        shooterFlywheels.runVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        shooterFlywheels.stop();
    }
}