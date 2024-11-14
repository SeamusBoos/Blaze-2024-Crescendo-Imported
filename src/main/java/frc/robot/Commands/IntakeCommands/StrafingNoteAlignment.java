package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class StrafingNoteAlignment extends Command {
    private final IntakeLimelight intakeLL;
    private final DriveSubsystemSwerve robotDrive;

    int driverControllerPort = ConstJoysticks.kDriverControllerPort;
    private final XboxController driverController = new XboxController(driverControllerPort);
    private double target = 0;
    private PIDController ThetaController;
    private double ThetaPower = 0;


    public StrafingNoteAlignment(IntakeLimelight LLIntake, DriveSubsystemSwerve driveSS){
        intakeLL = LLIntake;
        robotDrive = driveSS;

        ThetaController = new PIDController(1/75, 0, 0);
        
        addRequirements(robotDrive);
    }

    public void initialize() {
        ThetaController.setSetpoint(target);
    }

    @Override
    public void execute() {
        double NoteX = intakeLL.getX();
        ThetaPower = ThetaController.calculate(NoteX);

        if (intakeLL.getV()==1){
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), ThetaPower, false, true);
        } else {
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband), -MathUtil.applyDeadband(driverController.getRightX(), ConstJoysticks.kDriveDeadband), ConstMaxSwerveDrive.DriveConstants.kFieldCentric, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
//TODO make it strafe instead of turning