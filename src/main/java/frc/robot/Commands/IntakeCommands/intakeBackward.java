package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;
import frc.robot.Subsystems.Intake;

public class intakeBackward extends Command {
    private final Intake intake;

    public intakeBackward(Intake subsystem){
        intake = subsystem;
        addRequirements(subsystem); 
    }

    public void initialize() {
        intake.setVelocity(-ConstIntake.kIntakeVelocity);
    }

    @Override
    public void execute() {
        intake.runVel();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
