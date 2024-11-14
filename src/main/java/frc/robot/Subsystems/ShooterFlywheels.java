package frc.robot.Subsystems;

import frc.Mechanisms.configurationTypes.TwoMotorSSwithSparkMax;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class ShooterFlywheels extends TwoMotorSSwithSparkMax {

  public ShooterFlywheels() {
    super(ConstShooter.kMotorPortWheel1, ConstShooter.kMotorPortWheel2, 
          ConstShooter.invertedWheel1, ConstShooter.invertedWheel2, 
          ConstShooter.velFactorWheel1, ConstShooter.velFactorWheel2, "Flywheels");
      }
}