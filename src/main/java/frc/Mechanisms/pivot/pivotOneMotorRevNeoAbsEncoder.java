package frc.Mechanisms.pivot;

import frc.Mechanisms.configurationTypes.OneMotorSSwithSparkMaxAbsEncoder;

public class pivotOneMotorRevNeoAbsEncoder extends OneMotorSSwithSparkMaxAbsEncoder {

  public pivotOneMotorRevNeoAbsEncoder(int motorCanId, boolean inverted, double velConvFactor, String name, double P, double I, double D) {
    super(motorCanId, inverted, velConvFactor, name, P, I, D);
  }
}
