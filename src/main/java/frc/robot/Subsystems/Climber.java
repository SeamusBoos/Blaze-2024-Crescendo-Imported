package frc.robot.Subsystems;

import frc.Mechanisms.configurationTypes.TwoMotorSSwithSparkMax;
import frc.robot.SubmoduleSubsystemConstants.ConstClimber;

public class Climber extends TwoMotorSSwithSparkMax {

  public Climber() {
    super(ConstClimber.kMotorPortClimber1, ConstClimber.kMotorPortClimber2,
          ConstClimber.invertedMotor1, ConstClimber.invertedMotor2,
          ConstClimber.velFactorMotor1, ConstClimber.velFactorMotor2, "Climber");
    
  }
}