package frc.robot.Commands.ShooterAndPivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Mechanisms.sensorTypes.Limelight;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import frc.robot.Subsystems.CarriageBelt;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.ShooterFlywheels;
import frc.robot.Subsystems.ShooterPitch;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class AutoAlignOnly extends Command {
    
    ShooterPitch pitch;
    ShooterLimelight shooterLL;
    ShooterFlywheels flywheels;
    double pitchError;
    double thetaError;

    private final DriveSubsystemSwerve robotDrive;
    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double thetaPower;
    private PIDController thetaController;
    private double target = Limelight.alignmentConstants.AlignedSpeaker;

    public AutoAlignOnly(ShooterPitch shooterPitch, ShooterFlywheels shooterFlywheels, ShooterLimelight LLShooter, DriveSubsystemSwerve driveSS){
        pitch = shooterPitch;
        shooterLL = LLShooter;
        robotDrive = driveSS;
        flywheels = shooterFlywheels;

        thetaController = new PIDController(0.25, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        addRequirements(pitch, flywheels, shooterLL, robotDrive);
    }

    public void initialize(){
        thetaController.setSetpoint(target);
        ConstShooter.NoteInRobot = false;
    }

    public void execute(){
        pitchError = shooterLL.getPredictedPivot()-pitch.getAbsPosition();
        thetaError = target-shooterLL.getCamXInches();
        flywheels.setTargetVelocity(ConstShooter.defVelocity);
        flywheels.runVelocity();

        thetaPower = thetaController.calculate(shooterLL.getCamXMeters());

        if(shooterLL.getV()==1){
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), 
          ConstJoysticks.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
          thetaPower, true, true);
          pitch.seekPosition(shooterLL.getPredictedPivot());
          } else {
            robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
            pitch.seekPosition(ConstShooter.upperLimit);
          }
    }

    public void end(boolean interrupted){
        pitch.stop();
        flywheels.stop();
    }
}