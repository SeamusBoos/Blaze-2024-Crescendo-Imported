package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
// import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Subsystems.ShooterPitch;

public class Pass extends Command {

    private final ShooterFlywheels flywheels; 
    private final ShooterPitch pitch;
    private final Feeder feeder;
    private final CarriageBelt carriage;

    public Pass(ShooterFlywheels flywheel, ShooterPitch pitch, Feeder feeder, CarriageBelt carriage){
        this.flywheels = flywheel;
        this.pitch = pitch;
        this.feeder = feeder;
        this.carriage = carriage;
        addRequirements(flywheel, pitch, feeder, carriage);
    }

    public void initialize() {
        flywheels.setTargetVelocity(3000);
        feeder.setSpeed(-0.5);
        carriage.setSpeed(0.5);
    }

    @Override
    public void execute() {
        flywheels.setTargetVelocity(3000);
        flywheels.runVelocity();
        if(flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
        } else {
            feeder.stop();
            carriage.stop();
        }

        if(pitch.getSetpoint() >= ConstShooter.lowerLimit - 0.01){
            pitch.seekPosition(ConstShooter.lowerLimit);
        } else {
            pitch.seekPosition(ConstShooter.passPitch);
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
        carriage.stop();
        ConstShooter.NoteInRobot = false;
    }
}
