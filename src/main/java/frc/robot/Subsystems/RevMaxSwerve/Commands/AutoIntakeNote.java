package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;
import frc.robot.Sensors.FrontLimelight.IntakeLimelight;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.CardinalConstants;

/**
 * Command to reorient the robot to face forward.
 */
public class AutoIntakeNote extends Command {
  private final DriveSubsystemSwerve robotDrive;
  private final IntakeLimelight limelight;
  private final Intake intake;

  private double thetaPower;
  private double YPower;
  private PIDController thetaController;
  private PIDController YController;

  private double target;
  private int intakeSpeed = 4000;

  /**
   * Add subsystem and configure PID controller.
   *
   * @param subsystem drive subsystem
   */
  public AutoIntakeNote(DriveSubsystemSwerve subsystem, IntakeLimelight LLsubsystem, Intake intakeSubsystem) {
    robotDrive = subsystem;
    limelight = LLsubsystem;
    intake = intakeSubsystem;

    addRequirements(subsystem, LLsubsystem, intakeSubsystem);

    thetaController = new PIDController(0.005, CardinalConstants.CardinalI, CardinalConstants.CardinalD);
    YController = new PIDController(0.005, CardinalConstants.CardinalI, CardinalConstants.CardinalD);
    target = 0;
  }

  @Override
  public void initialize() {
    thetaController.setSetpoint(target);
    YController.setSetpoint(target);
  }

  @Override
  public void execute() {
    if (limelight.getV() == 1) {
      intake.setSpeed(intakeSpeed);
      intake.runVel();

      double NoteX = limelight.getX();
      thetaPower = thetaController.calculate(NoteX);
      YPower = YController.calculate(NoteX);
      robotDrive.drive(0.25, YPower, thetaPower, false, true);
    } else {
      robotDrive.drive(0, 0, 0, false, true);
      intake.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
