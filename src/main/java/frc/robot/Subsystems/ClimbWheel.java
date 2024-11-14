package frc.robot.Subsystems;

import frc.Mechanisms.rollers.*;
import frc.robot.SubmoduleSubsystemConstants.ConstClimbWheel;

public class ClimbWheel extends rollerOneMotorRevNeo {

  public ClimbWheel() {
    super(ConstClimbWheel.kMotorPortClimbWheel, ConstClimbWheel.inverted, ConstClimbWheel.wheelVelFactor, "Climb Wheel");
  }
}