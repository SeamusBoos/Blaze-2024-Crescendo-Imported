package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
// import frc.robot.Subsystems.ShooterPitch;

public class shoot extends Command {

    private final ShooterFlywheels flywheels; 
    // private final ShooterPitch pitch;
    private final Feeder feeder;
    private final CarriageBelt carriage;

    public shoot(ShooterFlywheels flywheel, /*ShooterPitch pitch,*/ Feeder feeder, CarriageBelt carriage){
        this.flywheels = flywheel;
        // this.pitch = pitch;
        this.feeder = feeder;
        this.carriage = carriage;

        addRequirements(flywheels, feeder, carriage);
    }

    public void initialize() {
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        feeder.setSpeed(-0.5);
        carriage.setSpeed(0.5);
    }

    @Override
    public void execute() {
        ConstShooter.NoteInRobot = false;
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
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
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
        carriage.stop();

    }
}
