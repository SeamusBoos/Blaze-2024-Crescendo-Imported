package frc.robot.SubmoduleSubsystemConstants;

public class ConstClimber {
    //Climber motor ports
    public static final int kMotorPortClimber1 = 13; //left
    public static final int kMotorPortClimber2 = 6; //right

    //Does the motor need to be inverted?
    public static final boolean invertedMotor1 = true;
    public static final boolean invertedMotor2 = false;

    //Velocity Conversion Factor
    public static final double velFactorMotor1 = 1.0;
    public static final double velFactorMotor2 = 1.0;

    // PID Controller Constants
  public static final double kP = 6e-4;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;

  //Max extention and retraction of the climber
  public static final double maxExt = 40;
  public static final double minExt = 0;

  //Speeds and Limits
  public static final double climbPower = 1;
}
