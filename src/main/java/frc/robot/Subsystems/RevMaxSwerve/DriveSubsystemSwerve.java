package frc.robot.Subsystems.RevMaxSwerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMode;
import frc.robot.SubmoduleSubsystemConstants.ConstProperties;

/**
 * Base drive subsystem class. Configures Swerve modules and gyro.
 */
public class DriveSubsystemSwerve extends SubsystemBase {
  // Create MAXSwerveModules
  // IMPORTANT - Redefine each module based on which motor controller you are
  // using
  private final MAXSwerveModuleWithSparkFlex frontLeft = new MAXSwerveModuleWithSparkFlex(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);
  private final MAXSwerveModuleWithSparkFlex frontRight = new MAXSwerveModuleWithSparkFlex(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);
  private final MAXSwerveModuleWithSparkFlex rearLeft = new MAXSwerveModuleWithSparkFlex(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);
  private final MAXSwerveModuleWithSparkFlex rearRight = new MAXSwerveModuleWithSparkFlex(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS navxGyro = new AHRS();
  private double currHeading;
  private double absHeading360 = 0;
  private double absHeading180 = 0;

  private double tolerance = 2.0;
  private double rotSpeed = 0;
  private double distToTarget = 0;

  // Drive Speeds
  private double FLDriveVel, FRDriveVel, BLDriveVel, BRDriveVel;

  // Variable Speed Settings
  private int speedMode = 1;

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  private SwerveControllerCommand cmd;
  private boolean trajectoryRunning = false;

  // Variables for Dashboard
  private ShuffleboardTab ssDriveTab;
  private ShuffleboardTab sbCompTab;
  private GenericEntry dashHeading, dashHeading180, dashHeading360, dashRotSpeed, dashPitch, dashFieldCentric, dashRotPtX;
  private GenericEntry dashSpeedMode, dashRotPtY, dashXPose, dashYPose, dashTrajRunning;
  private GenericEntry dashCHeading, dashCHeading180, dashCHeading360, dashCFieldCentric, dashCRotPtX;
  private GenericEntry dashCSpeedMode, dashCRotPtY;
  private GenericEntry sbFLDriveVel, sbFRDriveVel, sbBLDriveVel, sbBRDriveVel;

  // Advantage Scope Output
  StructArrayPublisher<SwerveModuleState> publisherStateMeasured = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Measured States", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> publisherStateDesired = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Desired States", SwerveModuleState.struct).publish();

  SwerveModuleState[] desiredStates = new SwerveModuleState[] { frontLeft.getState(),
      frontRight.getState(),
      frontLeft.getState(),
      frontLeft.getState() 
    };

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystemSwerve() {
    AutoBuilder.configureHolonomic(
        // Robot pose supplier
        this::getPose,
        // Method to reset odometry (will be called if your auto has a starting pose)
        this::resetOdometry,
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::getRobotRelativeSpeeds,
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        this::driveRobotRelative,
        // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new HolonomicPathFollowerConfig(
            // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0),
            // Rotation PID constants
            new PIDConstants(5.0, 0.0, 0.0),
            // Max module speed, in m/s
            6,
            // Drive base radius in meters. Distance from robot center to furthest module.
            0.4,
            // Default path replanning config. See the API for the options here
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    dashboardInit();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
    currHeading = getHeading();
    calcAbsHeading360();
    calcAbsHeading180();

    dashboardUpdate();

    publisherStateMeasured.set(getModuleStates());
    publisherStateDesired.set(desiredStates);

    FLDriveVel = frontLeft.getState().speedMetersPerSecond;
    FRDriveVel = frontRight.getState().speedMetersPerSecond;
    BLDriveVel = rearLeft.getState().speedMetersPerSecond;
    BRDriveVel = rearRight.getState().speedMetersPerSecond;
  }

  // SWERVE KINEMATIC FUNCTIONS AND VARIABLES
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /**
   * Retrieves chassis speed based on swerve module states.
   * Retrieves chassis speed based on swerve module states.
   *
   * @return Chassis speed of the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public double getRotation(){
    return currentRotation;
  }

  public void setRotation(double rotation){
    currentRotation = rotation;
  }

  public Translation2d setRotPoint(double x, double y) {
    Translation2d rot = new Translation2d(-y*DriveConstants.kWheelBase, -x*DriveConstants.kTrackWidth);
    DriveConstants.rotPt = rot;
    return rot;
  }

  // DRIVING FUNCTIONS
  /**
   * Robotcentric drive.
   *
   * @param speeds desired chassis speed
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, (getSpeed()));

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speedX        Speed of the robot in the x direction (forward).
   * @param speedY        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double speedX, double speedY, double rot, boolean fieldRelative, boolean rateLimit) {

    double speedCommandedX;
    double speedCommandedY;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(speedY, speedX);
      double inputTranslationMag = Math.sqrt(Math.pow(speedX, 2) + Math.pow(speedY, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        // some high number that means the slew rate is effectively instantaneous
        directionSlewRate = 2000.0; 
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = swerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = swerveUtils.StepTowardsCircular(currentTranslationDir, 
            inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        // some small number to avoid floating-point errors with equality checking
        if (currentTranslationMag > 1e-4) { 
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = swerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = swerveUtils.StepTowardsCircular(currentTranslationDir, 
            inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      speedCommandedX = currentTranslationMag * Math.cos(currentTranslationDir);
      speedCommandedY = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);


    } else {
      speedCommandedX = speedX;
      speedCommandedY = speedY;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double speedDeliveredX = speedCommandedX * (getSpeed());
    double speedDeliveredY = speedCommandedY * (getSpeed());
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(speedDeliveredX, speedDeliveredY, rotDelivered,
                Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(speedDeliveredX, speedDeliveredY, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, (getSpeed()));

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    desiredStates = swerveModuleStates.clone();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speedX        Speed of the robot in the x direction (forward).
   * @param speedY        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double speedX, double speedY, double rot, boolean fieldRelative, boolean rateLimit, Translation2d centerOfRot) {

    double speedCommandedX;
    double speedCommandedY;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(speedY, speedX);
      double inputTranslationMag = Math.sqrt(Math.pow(speedX, 2) + Math.pow(speedY, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        // some high number that means the slew rate is effectively instantaneous
        directionSlewRate = 2000.0; 
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = swerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = swerveUtils.StepTowardsCircular(currentTranslationDir, 
            inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        // some small number to avoid floating-point errors with equality checking
        if (currentTranslationMag > 1e-4) { 
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = swerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir = swerveUtils.StepTowardsCircular(currentTranslationDir, 
            inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      speedCommandedX = currentTranslationMag * Math.cos(currentTranslationDir);
      speedCommandedY = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);


    } else {
      speedCommandedX = speedX;
      speedCommandedY = speedY;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double speedDeliveredX = speedCommandedX * (getSpeed());
    double speedDeliveredY = speedCommandedY * (getSpeed());
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(speedDeliveredX, speedDeliveredY, rotDelivered,
                Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(speedDeliveredX, speedDeliveredY, rotDelivered), centerOfRot);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, (getSpeed()));

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);

    desiredStates = swerveModuleStates.clone();
  }

  private double getSpeed() {
    switch (speedMode) {
      case 0:
        return DriveConstants.kMaxSlowMPS;
      case 2:
        return DriveConstants.kMaxSpeedMetersPerSecond;
      case 3:
        return DriveConstants.kMaxGrannyMPS;
      default:
        return DriveConstants.kMaxDefaultMPS;
    }
  }

  public double getPitch() {
    return navxGyro.getPitch();
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels into orientation to go straight forward.
   */
  public void setStraight() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Set state of swerve modules.
   *
   * @param desiredStates desired states of each swerve module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, (getSpeed()));
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public void setFieldcentric(boolean fieldcentric) {
    DriveConstants.kFieldCentric = fieldcentric;
  }

  public void setRightStickNormalMode(boolean rightstickmode) {
    DriveConstants.kRightStickNormalMode = rightstickmode;
  }
  
  /**
   * Get swerve module states.
   *
   * @return array with current state of each swerve module
   */

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] { frontLeft.getState(), 
        frontRight.getState(), 
        rearLeft.getState(),
        rearRight.getState() };
  }

  public SwerveModuleState[] getDesiredModuleStates() {
    return desiredStates;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  public void setSpeedMode(Integer mode) {
    speedMode = mode;
  }

  // GYRO and HEADING HELPER FUNCTIONS
    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(navxGyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navxGyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /**
   * Calculates and absolute heading of gyro from 0 -> 360 going CCW.
   * Front = 0, Right = 90, Back = 180, Left = 270
   *
   * @return absolute 360 heading
   */
  private double calcAbsHeading360() {
    absHeading360 = (currHeading >= 0 ? currHeading % 360 : 360 + (currHeading % 360));
    return absHeading360;
  }

  /**
   * Calculates and absolute heading of gyro from -180 -> 180.
   * Front = 0, Right = 90, Back = +-180, Left = -90
   *
   * @return absolute heading from -180 to 180
   */
  private double calcAbsHeading180() {
    absHeading180 = (absHeading360 <= 180 ? absHeading360 % 180 : -(180 - (absHeading360 % 180)));
    return absHeading180;
  }

  public double getAbsHeading180(){
    return absHeading180;
  }

  public double getAbsHeading360(){
    return absHeading360;
  }

  /**
   * rotate robot to a targeted angle. 
   *
   * @param target desired heading to turn to
   * @param speedX speed
   * @param speedY speed
   */
  public void rotateToAngle(double target, double speedX, double speedY) {
    distToTarget = ((target - absHeading360) + 360.0) % 360.0;
    if (distToTarget < 180) {
      rotSpeed = distToTarget / 180.0;
    } else {
      rotSpeed = -((360.0 - distToTarget) / 180.0);
    }

    if (Math.abs(rotSpeed) < (tolerance / 360.0)) {
      rotSpeed = 0;
    }

    drive(speedX, speedY, rotSpeed, true, true);
  }

  /**
   * Determine which 90 degree side is closest to where the robot is right now.
   *
   * @return closest heading
   */
  public double findNearest90() {
    double[] front = { -30, 0, 30 };
    double[] right = { 60, 90, 120 };
    double[] back = { -150, 180, 150 };
    double[] left = { -120, -90, -60 };

    if (absHeading180 > front[0] && absHeading180 < front[2]) {
      return front[1];
    } else if (absHeading180 > right[0] && absHeading180 < right[2]) {
      return right[1];
      // TODO
    } else if (absHeading180 > back[0]) {

    } else if (absHeading180 > left[0] && absHeading180 < left[2]) {
      return left[1];
    }
    return 0.0;
  }

  /** 
   * Zeroes the heading of the robot. 
   * */
  public void zeroHeading() {
    navxGyro.reset();
    navxGyro.setAngleAdjustment(0);
  }

  // TRAJECTORY HELPER FUNCTIONS
  public boolean isTrajectoryRunning() {
    return trajectoryRunning;
  }

  public Command getTrajectoryCommand() {
    return cmd == null ? new WaitCommand(0.01) : cmd;
  }

  // DASHBOARD HELPER FUNCTIONS
  private void dashboardInit() {
    if (ConstMode.drivetrainDebug) {
      ssDriveTab = Shuffleboard.getTab("Drivetrain");
      dashSpeedMode = ssDriveTab.add("Speed Mode", speedMode).getEntry();
      dashHeading = ssDriveTab.add("Heading", getHeading()).getEntry();
      dashHeading180 = ssDriveTab.add("Heading180", absHeading180).getEntry();
      dashHeading360 = ssDriveTab.add("Heading360", absHeading360).getEntry();
      dashRotSpeed = ssDriveTab.add("Rotate Speed", rotSpeed).getEntry();
      dashPitch = ssDriveTab.add("Pitch", getPitch()).getEntry();
      dashFieldCentric = ssDriveTab.add("FieldCentric", DriveConstants.kFieldCentric).getEntry();
      dashRotPtX = ssDriveTab.add("RotationPtX", DriveConstants.rotPt.getX()).getEntry();
      dashRotPtY = ssDriveTab.add("RotationPtY", DriveConstants.rotPt.getY()).getEntry();

      dashXPose = ssDriveTab.add("X Coordinate", getPose().getX()).getEntry();
      dashYPose = ssDriveTab.add("Y Coordinate", getPose().getY()).getEntry();

      trajectoryRunning = cmd != null ? cmd.isScheduled() : false;

      dashTrajRunning = ssDriveTab.add("is scheduled", trajectoryRunning).getEntry();

      sbFLDriveVel = ssDriveTab.add("FLDriveVel", FLDriveVel).getEntry();
      sbFRDriveVel = ssDriveTab.add("FRDriveVel", FRDriveVel).getEntry();
      sbBLDriveVel = ssDriveTab.add("BLDriveVel", BLDriveVel).getEntry();
      sbBRDriveVel = ssDriveTab.add("BRDriveVel", BRDriveVel).getEntry();

      // ssDriveTab.add("front left rotation", getModuleStates()[0].angle.getDegrees());
      // ssDriveTab.add("front right rotation", getModuleStates()[1].angle.getDegrees());
      // ssDriveTab.add("rear left rotation", getModuleStates()[2].angle.getDegrees());
      // ssDriveTab.add("rear right rotation", getModuleStates()[3].angle.getDegrees());
    }
    sbCompTab = Shuffleboard.getTab(ConstProperties.CompDashboard.COMPDASHNAME_STRING);
    dashCSpeedMode = sbCompTab.add("Speed Mode", speedMode).getEntry();
    dashCHeading = sbCompTab.add("Heading", getHeading()).getEntry();
    dashCHeading180 = sbCompTab.add("Heading180", absHeading180).getEntry();
    dashCHeading360 = sbCompTab.add("Heading360", absHeading360).getEntry();
    dashCFieldCentric = sbCompTab.add("FieldCentric", DriveConstants.kFieldCentric).getEntry();
  }

  public void dashboardUpdate(){
    if (ConstMode.drivetrainDebug) {
      dashSpeedMode.setInteger(speedMode);
      dashHeading.setDouble(getHeading());
      dashHeading180.setDouble(absHeading180);
      dashHeading360.setDouble(absHeading360);
      dashRotSpeed.setDouble(rotSpeed);
      dashPitch.setDouble(getPitch());
      dashFieldCentric.setBoolean(DriveConstants.kFieldCentric);
      dashRotPtX.setDouble(DriveConstants.rotPt.getX());
      dashRotPtY.setDouble(DriveConstants.rotPt.getY());

      dashXPose.setDouble(getPose().getX());
      dashYPose.setDouble(getPose().getY());

      trajectoryRunning = cmd != null ? cmd.isScheduled() : false;

      dashTrajRunning.setBoolean(trajectoryRunning);

      sbFLDriveVel.setDouble(FLDriveVel);
      sbFRDriveVel.setDouble(FRDriveVel);
      sbBLDriveVel.setDouble(BLDriveVel);
      sbBRDriveVel.setDouble(BRDriveVel);

      // ssDriveTab.add("front left rotation", getModuleStates()[0].angle.getDegrees());
      // ssDriveTab.add("front right rotation", getModuleStates()[1].angle.getDegrees());
      // ssDriveTab.add("rear left rotation", getModuleStates()[2].angle.getDegrees());
      // ssDriveTab.add("rear right rotation", getModuleStates()[3].angle.getDegrees());
    
    }
    dashCSpeedMode.setInteger(speedMode);
    dashCHeading.setDouble(getHeading());
    dashCHeading180.setDouble(absHeading180);
    dashCHeading360.setDouble(absHeading360);
    dashCFieldCentric.setBoolean(DriveConstants.kFieldCentric);
    
  }
}