package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;

public class SpinUpWithPivot extends Command {
    
    ShooterPitch pitch;
    ShooterFlywheels flywheels;
    ShooterLimelight shooterLL;
    double pitchError;



    public SpinUpWithPivot(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, ShooterLimelight LLShooter){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        flywheels = shooterFlywheels;

        addRequirements(pitch, flywheels);
    }

    public void initialize(){
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        ConstShooter.NoteInRobot = false;
    }

    public void execute(){
        pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();
        pitch.seekPosition(shooterLL.getPredictedPivot());
    }

    public void end(boolean interrupted){
        flywheels.stop();
        pitch.stop();
    }
}