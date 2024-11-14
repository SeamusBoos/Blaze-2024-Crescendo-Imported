package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class StayReady extends Command {

    Timer amplifiedTimer = new Timer();
    
    public StayReady(){

    }

    public void initialize(){
        ConstShooter.stayReady = true;
        amplifiedTimer.reset();
        amplifiedTimer.start();
    }

    @Override
    public void execute(){
        if(amplifiedTimer.get()>15){
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted){
        ConstShooter.stayReady = false;
    }
}
