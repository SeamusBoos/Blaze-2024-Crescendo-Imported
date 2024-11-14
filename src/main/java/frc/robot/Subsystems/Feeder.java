package frc.robot.Subsystems;

import frc.Mechanisms.rollers.*;
import frc.robot.SubmoduleSubsystemConstants.ConstFeeder;

public class Feeder extends rollerOneMotorRevNeo {

  public Feeder() {
    super(ConstFeeder.kMotorPort, ConstFeeder.kInverted, ConstFeeder.velFactor, "Feeder");
    setSpeed(0);
    setVelocity(0);
  }
}