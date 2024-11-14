package frc.robot.SubmoduleSubsystemConstants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ConstMaxSwerveDrive {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 7.22;
    public static final double kMaxDefaultMPS = 4.8;
    public static final double kMaxSlowMPS = 2;
    public static final double kMaxGrannyMPS = 0.4;
    public static final double kMaxAngularSpeed = 10; // radians per second

    public static final double kDirectionSlewRate = 3 * Math.PI; // radians per second
    public static final double kMagnitudeSlewRate = 8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 120.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kRevMaxSwerveOffset = Units.inchesToMeters(1.75);
    public static final double kTrackWidth = Units.inchesToMeters(28);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(30);

    // Variables for location of the swerve drive in relation to the center of the robot
    // TODO need to update for Crescendo Robot
    public static double kFrontLeftX = (kWheelBase / 2) - kRevMaxSwerveOffset;
    public static double kFrontLeftY = (kTrackWidth / 2) - kRevMaxSwerveOffset;
    public static double kFrontRightX = (kWheelBase / 2) - kRevMaxSwerveOffset;
    public static double kFrontRightY = -((kTrackWidth / 2) - kRevMaxSwerveOffset);
    public static double kBackLeftX = -((kWheelBase / 2) - kRevMaxSwerveOffset);
    public static double kBackLeftY = (kTrackWidth / 2) - kRevMaxSwerveOffset;
    public static double kBackRightX = -((kWheelBase / 2) - kRevMaxSwerveOffset);
    public static double kBackRightY = -((kTrackWidth / 2) - kRevMaxSwerveOffset);

    // Locations for the swerve drive modules relative to the robot center.
    private static final Translation2d m_frontLeftLocation = new Translation2d(kFrontLeftX, kFrontLeftY);
    private static final Translation2d m_frontRightLocation = new Translation2d(kFrontRightX, kFrontRightY);
    private static final Translation2d m_backLeftLocation = new Translation2d(kBackLeftX, kBackLeftY);
    private static final Translation2d m_backRightLocation = new Translation2d(kBackRightX, kBackRightY);

  // Creating my kinematics object using the module locations
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
      );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI /2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI /2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 18;
    public static final int kRearLeftDrivingCanId = 19;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 20;

    public static final int kFrontLeftTurningCanId = 16;
    public static final int kRearLeftTurningCanId = 17;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = true;

    public static boolean kFieldCentric = true;

    public static boolean kRightStickNormalMode = true;

    public static Translation2d rotPt = new Translation2d(0, 0);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;
    public static final int kDrivingSpurGearTeeth = 20;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.071; //0.071M //0.0762 - DEFAULT (3in)
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 16 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * kDrivingSpurGearTeeth) / (kDrivingMotorPinionTeeth * 16);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 7.22;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class CardinalConstants {
    public static final double CardinalP = 1/75.0;
    public static final double CardinalI = 0;
    public static final double CardinalD = 0;
  }

  public static final class GamePieceAlignmentConstants {
    public static final double P = 0.01;
    public static final double I = 0;
    public static final double D = 0.0005;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
