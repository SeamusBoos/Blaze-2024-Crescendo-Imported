package frc.robot.SubmoduleSubsystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class ConstShooter {

  // Constants for the shooter
  public static final int kMotorPortWheel1 = 15; //Bottom
  public static final int kMotorPortWheel2 = 8;  //top
  // public static final int kMotorPortBelt = 7;
  public static final int kPivot = 12;

  // Does the motor need to be inverted?
  public static final boolean invertedWheel1 = true;
  public static final boolean invertedWheel2 = false;
  public static final boolean invertedPivot = false;

  // Velocity Conversion Factor
  public static final double velFactorWheel1 = 1.0;
  public static final double velFactorWheel2 = 1.0;
  // public static final double velFactorBelt = 1.0;
  public static final double velFactorPivot = 1.0;


  // PID Controller Constants
  public static final double kP = 6e-4;
  public static final double kI = 5e-9;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kFF = 1.0 / 5700.0;
  public static final double kMaxOut = 1;
  public static final double kMinOut = -1;
  public static final double maxRPM = 5700;

  // PID Controller Constants
  public static final double pivotkP = 6e-4;
  public static final double pivotkI = 5e-9;
  public static final double pivotkD = 0;
  public static final double pivotkIz = 0;
  public static final double pivotkFF = 1.0 / 5700.0;
  public static final double pivotkMaxOut = 1;
  public static final double pivotkMinOut = -1;
  public static final double pivotmaxRPM = 5700;

  // Speeds and Limits
  public static final double defVelocity = 4000;
  public static final double pivotMaxSpeed = 0.2; //0.2

  public static final double lowerLimit = 0.575; //0.6 0.55
  public static final double upperLimit = 0.275;

  public static final double passPitch = 0.38;

  public static boolean NoteInRobot = false;

  public static boolean stayReady = false;
}
