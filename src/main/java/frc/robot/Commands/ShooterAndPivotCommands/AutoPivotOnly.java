package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;

public class AutoPivotOnly extends Command {
    
    ShooterPitch pitch;
    ShooterFlywheels flywheels;
    Feeder feeder;
    CarriageBelt carriage;
    ShooterLimelight shooterLL;
    double pitchError;



    public AutoPivotOnly(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, Feeder feederSS, CarriageBelt belt, ShooterLimelight LLShooter){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        flywheels = shooterFlywheels;
        feeder = feederSS;
        carriage = belt;

        addRequirements(pitch, flywheels, feeder, carriage);
    }

    public void initialize(){
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        feeder.setSpeed(-0.5);
        carriage.setSpeed(0.5);
        ConstShooter.NoteInRobot = false;
    }

    public void execute(){
        pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();
        pitch.seekPosition(shooterLL.getPredictedPivot());
        
        if(Math.abs(pitchError)<0.075 && flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
        } else {
            feeder.stop();
            carriage.stop();
        }
    }

    public void end(boolean interrupted){
        flywheels.stop();
        pitch.stop();
        feeder.stop();
        carriage.stop();
    }
}