package frc.robot.SubmoduleSubsystemConstants;

public class ConstFeeder {
    //Carriage & Elevator motor ports
    public static final int kMotorPort = 7;

    //Does the motor need to be inverted?
    public static final boolean kInverted = true;

    //Velocity Conversion Factor
    public static final double velFactor = 1.0;

    public static final double defaultVelocity = 2000;

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
