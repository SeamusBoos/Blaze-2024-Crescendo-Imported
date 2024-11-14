package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterPitch;

public class TunePitch extends Command {

    XboxController subsystemController = new XboxController(3);

    ShooterPitch pitch;
    double defaultPos = ConstShooter.upperLimit;
    double Pose = defaultPos;
    
    public TunePitch(ShooterPitch pitch){
        this.pitch = pitch;
        
        addRequirements(pitch);
    }

public void initialize() {
    // pitch.seekPosition(defaultPos);
    }

    @Override
    public void execute() {
        if(subsystemController.getAButtonPressed()){
            Pose += 0.001;
        } else if(subsystemController.getYButtonPressed()){
            Pose -= 0.001;
        }

        if(Pose<ConstShooter.upperLimit){
            Pose = ConstShooter.upperLimit;
        } else if(Pose>ConstShooter.lowerLimit){
            Pose = ConstShooter.lowerLimit;
        }
        pitch.seekPosition(Pose);
    }

}
