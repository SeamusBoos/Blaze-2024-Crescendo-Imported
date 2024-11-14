package frc.robot.SubmoduleSubsystemConstants;

public class ConstElevator {
    public static final int kMotorPortElevator1 = 14; //left
    public static final int kMotorPortElevator2 = 4; //right

    //Does the motor need to be inverted?
    public static final boolean kElevator1Inverted = false;
    public static final boolean kElevator2Inverted = true;

    //Velocity Conversion Factor
    public static final double velFactorElevator1 = 1.0;
    public static final double velFactorElevator2 = 1.0;

    // PID Controller Constants
  public static final double kP = 6e-4;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;

  //Speeds and Limits
  public static final double raisedPos = 75; //80 is true max
  public static final double loweredPos = 0;
  public static final double elevatorRiseSpeed = 1;
  public static final double elevatorFinishSpeed = 0.2;
  public static final double elevatorFallSpeed = 1;
  public static final double elevatorSlowZoneUp = 67.5;
  public static final double elevatorSlowZoneDown = 7.5;
}
