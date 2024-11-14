package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.ShooterFlywheels;
// import frc.robot.Subsystems.ShooterPitch;

public class FullPassthrough extends Command {

    private final ShooterFlywheels flywheels; 
    // private final ShooterPitch pitch;
    private final Feeder feeder;
    private final CarriageBelt carriage;
    private final Intake intake;

    public FullPassthrough(ShooterFlywheels flywheel, /*ShooterPitch pitch,*/ Feeder feeder, CarriageBelt carriage, Intake intake){
        this.flywheels = flywheel;
        // this.pitch = pitch;
        this.feeder = feeder;
        this.carriage = carriage;
        this.intake = intake;

        addRequirements(flywheels, feeder, carriage, intake);
    }

    public void initialize() {
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        feeder.setSpeed(-0.5);
        carriage.setSpeed(0.5);
        intake.setSpeed(ConstIntake.kIntakeSpeed);
    }

    @Override
    public void execute() {
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();
        // if(flywheels.atSpeed()){
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
            intake.setSpeed(ConstIntake.kIntakeSpeed);
            intake.runSpeed();
        // } else {
        //     feeder.stop();
        //     carriage.stop();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
        carriage.stop();
        intake.stop();
    }
}
