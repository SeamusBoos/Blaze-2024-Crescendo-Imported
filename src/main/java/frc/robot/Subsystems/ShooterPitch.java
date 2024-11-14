package frc.robot.Subsystems;

import frc.Mechanisms.pivot.*;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class ShooterPitch extends pivotOneMotorRevNeoAbsEncoder {

  public ShooterPitch() {
    super(ConstShooter.kPivot, ConstShooter.invertedPivot, ConstShooter.velFactorPivot, "Shot Pitch", 2.25, 0.1, 0);
  }
}