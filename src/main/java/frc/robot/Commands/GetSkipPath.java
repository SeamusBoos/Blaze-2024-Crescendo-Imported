package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstAuto;

public class GetSkipPath extends Command{
    
    public GetSkipPath(){

    }

    public void initialize(){

    }

    @Override
    public void execute(){
        if(ConstAuto.skipNextPath){
            end(true);
        } else {
            doNothing();
        }
    }

    @Override
    public void end(boolean interrupted){
        ConstAuto.skipNextPath = false;
    }

    private void doNothing(){

    }
}
