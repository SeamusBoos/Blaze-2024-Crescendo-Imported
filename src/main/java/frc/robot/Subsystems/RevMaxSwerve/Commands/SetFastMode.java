package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

/**
 * Change robot drive to fast mode.
 */
public class SetFastMode extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private DriveSubsystemSwerve robotDrive;

  public SetFastMode(DriveSubsystemSwerve subsystem) {
    robotDrive = subsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    robotDrive.setSpeedMode(2);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}