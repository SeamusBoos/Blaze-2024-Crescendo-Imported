package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class pitchSubwoofer extends Command {
    
    ShooterPitch pitch;


    public pitchSubwoofer(ShooterPitch shooterPitch){
        pitch = shooterPitch;

        addRequirements(pitch);
    }

    public void initialize(){

    }

    public void execute(){
        pitch.seekPosition(ConstShooter.upperLimit);
    }

    public void end(boolean interrupted){
        pitch.stop();
    }
}
