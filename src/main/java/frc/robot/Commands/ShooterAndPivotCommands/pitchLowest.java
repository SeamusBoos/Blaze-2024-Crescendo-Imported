package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class pitchLowest extends Command {
    
    ShooterPitch pitch;


    public pitchLowest(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){

    }

    public void execute(){
        pitch.seekPosition(ConstShooter.lowerLimit);
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}
