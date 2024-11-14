package frc.robot.SubmoduleSubsystemConstants;

public class ConstClimbWheel {
    //Climber motor ports
    public static final int kMotorPortClimbWheel = 10;

    //Does the motor need to be inverted?
    public static final boolean inverted = false;

    //Velocity Conversion Factor
    public static final double wheelVelFactor = 1.0;

    // PID Controller Constants
  public static final double kP = 6e-4;
  public static final double kI = 5e-9;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;
}
