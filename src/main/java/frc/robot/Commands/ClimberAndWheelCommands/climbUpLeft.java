package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.LimitSwitches.LeftArmDownSwitch;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;
import frc.robot.Subsystems.Climber;

public class climbUpLeft extends Command {
    private final Climber climber;
    private final LeftArmDownSwitch leftSwitch;

    public climbUpLeft(Climber subsystem, LeftArmDownSwitch Lswitch){
        climber = subsystem;
        leftSwitch = Lswitch;
        // addRequirements(subsystem);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        // climber.runMotor2Seperate(ConstClimber.climbPower);

        if (leftSwitch.getLimitReached() == false){
         climber.stopMotor2();
        } else {
         climber.runMotor2Seperate(ConstClimber.climbPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor2();
    }
}
