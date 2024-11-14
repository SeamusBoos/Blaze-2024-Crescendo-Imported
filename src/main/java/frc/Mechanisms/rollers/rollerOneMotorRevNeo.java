package frc.Mechanisms.rollers;

import frc.Mechanisms.configurationTypes.OneMotorSSwithSparkMax;

public class rollerOneMotorRevNeo extends OneMotorSSwithSparkMax {

  public rollerOneMotorRevNeo(int motorCanId, boolean inverted, double velConvFactor, String name) {
    super(motorCanId, inverted, velConvFactor, name);
  }
}
