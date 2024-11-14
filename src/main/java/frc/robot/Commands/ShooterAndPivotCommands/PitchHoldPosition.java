package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPitch;

public class PitchHoldPosition extends Command {
    
    ShooterPitch pitch;

    double holdingPos;


    public PitchHoldPosition(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){
        holdingPos = pitch.getAbsPosition();
    }

    public void execute(){
        pitch.seekPosition(holdingPos);
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}