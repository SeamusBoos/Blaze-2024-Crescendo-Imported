package frc.robot.Commands.ShooterAndPivotCommands.MiamiValleyPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPitch;

public class PitchAmp extends Command {
    
    ShooterPitch pitch;


    public PitchAmp(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){

    }

    public void execute(){
        pitch.seekPosition(0.4); //0.39
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}