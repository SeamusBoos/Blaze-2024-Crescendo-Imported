package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;

public class FullPassthroughBloop extends Command {

    private final ShooterFlywheels flywheels; 
    private final ShooterPitch pitch;
    private final Feeder feeder;
    private final CarriageBelt carriage;
    private final Intake intake;
    private final IntakeLimelight intakeLL;
    private final ShooterLimelight shooterLL;

    public FullPassthroughBloop(ShooterFlywheels flywheel, ShooterPitch pitch, Feeder feeder, CarriageBelt carriage, Intake intake, IntakeLimelight intakeLL, ShooterLimelight shooterLL){
        this.flywheels = flywheel;
        this.feeder = feeder;
        this.carriage = carriage;
        this.intake = intake;
        this.pitch = pitch;
        this.intakeLL = intakeLL;
        this.shooterLL = shooterLL;

        addRequirements(flywheels, feeder, carriage, intake, pitch);
    }

    public void initialize() {
        flywheels.setTargetVelocity(500);
        feeder.setSpeed(-0.5);
        carriage.setSpeed(0.5);
        intake.setSpeed(ConstIntake.kIntakeSpeed);
    }

    @Override
    public void execute() {
        pitch.seekPosition(ConstShooter.lowerLimit);
        flywheels.setTargetVelocity(500);
        flywheels.runVelocity();
            feeder.setSpeed(-0.5);
            feeder.runSpeed();
            carriage.setSpeed(0.5);
            carriage.runSpeed();
            intake.setSpeed(ConstIntake.kIntakeSpeed);
            intake.runSpeed();

            if(carriage.getVelocity()>100){
                intakeLL.setLedMode(3);
                shooterLL.setLedMode(3);
            } else {
                intakeLL.setLedMode(1);
                shooterLL.setLedMode(1);
            }
    }

    @Override
    public void end(boolean interrupted) {
        flywheels.stop();
        feeder.stop();
        carriage.stop();
        intake.stop();
    }
}
