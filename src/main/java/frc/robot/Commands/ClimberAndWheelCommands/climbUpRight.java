package frc.robot.Commands.ClimberAndWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.LimitSwitches.RightArmDownSwitch;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;
import frc.robot.Subsystems.Climber;

public class climbUpRight extends Command {
    private final Climber climber;
    private final RightArmDownSwitch rightSwitch;

    public climbUpRight(Climber subsystem, RightArmDownSwitch Rswitch){
        climber = subsystem;
        rightSwitch = Rswitch;
        // addRequirements(subsystem); 
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        // climber.runMotor1Seperate(ConstClimber.climbPower);


        if (rightSwitch.getLimitReached() == false){
            climber.stopMotor1();
        } else {
            climber.runMotor1Seperate(ConstClimber.climbPower);
        }    
    }

    @Override
    public void end(boolean interrupted) {
        climber.stopMotor1();
    }
}
