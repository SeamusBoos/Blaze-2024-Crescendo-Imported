package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class RotatingNoteAlignment extends Command {
    private final IntakeLimelight intakeLL;
    private final DriveSubsystemSwerve robotDrive;

    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double NoteX = 0;
    private double target = 0;
    private PIDController ThetaController;
    private double ThetaPower = 0;


    public RotatingNoteAlignment(IntakeLimelight LLIntake, DriveSubsystemSwerve driveSS){
        intakeLL = LLIntake;
        robotDrive = driveSS;

        ThetaController = new PIDController(0.01, 0, 0.005);
        
        addRequirements(robotDrive);
    }

    public void initialize() {
        ThetaController.setSetpoint(0);
    }

    @Override
    public void execute() {
        NoteX = intakeLL.getX();
        ThetaPower = ThetaController.calculate(NoteX);

        if (intakeLL.getV()==1){
            robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), ThetaPower, false, true);
        } else { 
            robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), false, true);
        }
        SmartDashboard.putNumber("Theta Power", ThetaPower);
        SmartDashboard.putNumber("NoteX", NoteX);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
