package frc.robot.Subsystems;

import frc.Mechanisms.rollers.*;
import frc.robot.SubmoduleSubsystemConstants.ConstCarriage;

public class CarriageBelt extends rollerOneMotorRevNeo {

  public CarriageBelt() {
    super(ConstCarriage.kMotorPortCarriage, ConstCarriage.kCarriageInverted, ConstCarriage.velFactorCarriage, "Carriage Belt");
  }
}