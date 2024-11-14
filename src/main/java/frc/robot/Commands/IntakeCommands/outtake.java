package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Intake;

public class outtake extends Command {
    private final Intake intake;
    private final CarriageBelt carriageBelt;

    public outtake(Intake intakeSS, CarriageBelt carriageSS){
        intake = intakeSS;
        carriageBelt = carriageSS;
        addRequirements(intake, carriageBelt); 
    }

    public void initialize() {
        intake.setSpeed(-0.5);
        carriageBelt.setSpeed(-0.5);
    }

    @Override
    public void execute() {
        intake.runSpeed();
        carriageBelt.runSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        carriageBelt.stop();
    }
}
