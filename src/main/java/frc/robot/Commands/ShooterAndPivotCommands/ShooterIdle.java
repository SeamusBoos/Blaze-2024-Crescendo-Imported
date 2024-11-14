package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.ShooterFlywheels;

public class ShooterIdle extends Command {

    private final ShooterFlywheels shooterFlywheels;
    
    public ShooterIdle(ShooterFlywheels flywheels){
        shooterFlywheels = flywheels;

        addRequirements(flywheels);
    }

    public void initialize(){
        shooterFlywheels.setTargetVelocity(2250);
    }

    @Override
    public void execute(){
        if(ConstShooter.stayReady){
        shooterFlywheels.setTargetVelocity(4000);
        shooterFlywheels.runVelocity();
        } else if(ConstShooter.NoteInRobot){
        shooterFlywheels.setTargetVelocity(2250);
        shooterFlywheels.runVelocity();
        } else {
        shooterFlywheels.stop();
        }
        SmartDashboard.putBoolean("Note Held", ConstShooter.NoteInRobot);
    }

    @Override
    public void end(boolean interrupted){
        shooterFlywheels.stop();
    }
}
