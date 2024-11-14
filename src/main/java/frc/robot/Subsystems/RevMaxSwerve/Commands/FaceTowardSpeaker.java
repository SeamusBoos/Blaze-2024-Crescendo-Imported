package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.BackLimelight.ShooterLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class FaceTowardSpeaker extends Command {
    private final DriveSubsystemSwerve robotDrive;
    private final ShooterLimelight shooterLimelight;

    private double thetaPower;
    private PIDController thetaController;

    public FaceTowardSpeaker(DriveSubsystemSwerve driveSubsystem, ShooterLimelight limelightSubsystem) {
        robotDrive = driveSubsystem;
        shooterLimelight = limelightSubsystem;

        addRequirements(driveSubsystem, limelightSubsystem);

        thetaController = new PIDController(0.005, CardinalConstants.CardinalI, CardinalConstants.CardinalD);
    }

    @Override
    public void initialize() {
        thetaController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (shooterLimelight.getV() == 1) {
            double tx = shooterLimelight.getX();

            thetaPower = thetaController.calculate(tx);
            robotDrive.drive(0, 0, thetaPower, false, true);
        }
        else {
            robotDrive.drive(0, 0, thetaPower, false, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
