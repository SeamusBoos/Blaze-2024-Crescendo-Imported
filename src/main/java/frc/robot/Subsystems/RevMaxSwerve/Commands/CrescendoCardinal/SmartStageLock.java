package frc.robot.Subsystems.RevMaxSwerve.Commands.CrescendoCardinal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

public class SmartStageLock extends Command {
    private final DriveSubsystemSwerve m_robotDrive;

    private double heading180, heading360;


    private final XboxController m_driverController = new XboxController(ConstJoysticks.kDriverControllerPort);


    private double thetaPower;
    private PIDController thetaController;

    private double target;
    
    public SmartStageLock(DriveSubsystemSwerve subsystem){
        m_robotDrive = subsystem;

        addRequirements(subsystem);

        thetaController = new PIDController(CardinalConstants.CardinalP, CardinalConstants.CardinalI, CardinalConstants.CardinalD);

        
    }

    @Override
    public void initialize() {
        heading180 = m_robotDrive.getAbsHeading180();
        heading360 = m_robotDrive.getAbsHeading360();
        if(heading180 >- 120 && heading180 <= 0){
            target = -60;
        } else if(heading180 <120 && heading180 >0){
            target = 60;
        } else if(heading360 >= 120){
            target = 180;
        }

        thetaController.setSetpoint(target);
    }

    @Override
    public void execute() {

        heading180 = m_robotDrive.getAbsHeading180();
        heading360 = m_robotDrive.getAbsHeading360();

        if(target==180){
            thetaPower = thetaController.calculate(heading360);
    
            m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                               -MathUtil.applyDeadband(m_driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                               thetaPower, true, true);
        } else {
            thetaPower = thetaController.calculate(heading180);
    
            if((heading180-target<-180)||(heading180-target>180)) {
            m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                               -MathUtil.applyDeadband(m_driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                               -thetaPower, true, true);
            } else {
            m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), ConstJoysticks.kDriveDeadband),
                               -MathUtil.applyDeadband(m_driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
                               thetaPower, DriveConstants.kFieldCentric, true);
            }
            }
        
    }

    @Override
  public void end(boolean interrupted) {
  }
}
