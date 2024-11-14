package frc.robot.Subsystems;

import frc.Mechanisms.rollers.*;
import frc.robot.SubmoduleSubsystemConstants.ConstIntake;

public class Intake extends rollerOneMotorRevNeo {

  public Intake() {
    super(ConstIntake.kIntakePort, ConstIntake.kIntakeInverted, ConstIntake.kIntakeVelFactor, "Intake");
    setSpeed(0);
    setVelocity(0);
  }
}