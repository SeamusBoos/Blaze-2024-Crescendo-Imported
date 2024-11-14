package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class FaceRight extends Command {
    private final DriveSubsystemSwerve robotDrive;

    private final XboxController driverController = new XboxController(ConstJoysticks.kDriverControllerPort);


    private double thetaPower;
    private PIDController thetaController;

    private double target;
    
    public FaceRight(DriveSubsystemSwerve subsystem){
        robotDrive = subsystem;

        addRequirements(subsystem);

        thetaController = new PIDController(CardinalConstants.CardinalP, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        target = -30; //-90;
    }

    @Override
    public void initialize() {
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        thetaController.setSetpoint(target);
    }

    @Override
    public void execute() {


        double heading = robotDrive.getAbsHeading180();

        

        thetaPower = thetaController.calculate(heading);

        if((heading-target<-180)||(heading-target>180)) {
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                           -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                           -thetaPower, true, true);
        } else {
        robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                           -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                           thetaPower, true, true);
        }
        
    }

    @Override
  public void end(boolean interrupted) {
  }
}
