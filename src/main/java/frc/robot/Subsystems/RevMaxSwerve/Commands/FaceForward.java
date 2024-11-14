package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.SubmoduleSubsystemConstants.ConstJoysticks;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;

/**
 * Command to reorient the robot to face forward.
 */
public class FaceForward extends Command {
  private final DriveSubsystemSwerve robotDrive;
  int driverControllerPort = ConstJoysticks.kDriverControllerPort;
  private final XboxController driverController = new XboxController(driverControllerPort);
  private double thetaPower;
  private PIDController thetaController;
  private double target;

  /**
   * Add subsystem and configure PID controller.
   *
   * @param subsystem drive subsystem
   */
  public FaceForward(DriveSubsystemSwerve subsystem) {
    robotDrive = subsystem;
    addRequirements(subsystem);
    thetaController = new PIDController(CardinalConstants.CardinalP, CardinalConstants.CardinalI, CardinalConstants.CardinalD);
    target = 0;
  }

  @Override
  public void initialize() {
    thetaController.setSetpoint(target);
  }

  @Override
  public void execute() {
    double heading = robotDrive.getAbsHeading180();
    thetaPower = thetaController.calculate(heading);
    robotDrive.drive(-MathUtil.applyDeadband(driverController.getLeftY(), 
          ConstJoysticks.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(), ConstJoysticks.kDriveDeadband),
          thetaPower, true, true);
  }

  @Override
  public void end(boolean interrupted) {
  }
}
