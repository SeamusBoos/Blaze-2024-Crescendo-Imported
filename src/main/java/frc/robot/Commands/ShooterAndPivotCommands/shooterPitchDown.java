package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class shooterPitchDown extends Command {
    private final ShooterPitch shooterPitch;

    public shooterPitchDown(ShooterPitch subsystem){
        shooterPitch = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        shooterPitch.setSpeed(-ConstShooter.pivotMaxSpeed);
    }

    @Override
    public void execute() {
        if (shooterPitch.getAbsPosition()<ConstShooter.lowerLimit && shooterPitch.getAbsPosition()>0){
            shooterPitch.run();
        }
        else{
            shooterPitch.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterPitch.stop();
    }
}
