package frc.robot.Subsystems;

import frc.Mechanisms.configurationTypes.TwoMotorSSwithSparkMax;
import frc.robot.SubmoduleSubsystemConstants.ConstElevator;

public class Elevator extends TwoMotorSSwithSparkMax {

  public Elevator() {
    super(ConstElevator.kMotorPortElevator1, ConstElevator.kMotorPortElevator2,
          ConstElevator.kElevator1Inverted, ConstElevator.kElevator2Inverted,
          ConstElevator.velFactorElevator1, ConstElevator.velFactorElevator2, "Elevator");
      }
}