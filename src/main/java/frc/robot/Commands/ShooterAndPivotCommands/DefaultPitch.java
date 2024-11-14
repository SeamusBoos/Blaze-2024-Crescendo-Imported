package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class DefaultPitch extends Command {
    
    ShooterPitch pitch;


    public DefaultPitch(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){

    }

    public void execute(){
        if(pitch.getSetpoint()==ConstShooter.lowerLimit){
            pitch.seekPosition(ConstShooter.lowerLimit);
        }else{
        pitch.seekPosition(ConstShooter.upperLimit);
        }
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}
