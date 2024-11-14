package frc.robot.Subsystems.RevMaxSwerve.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.RevMaxSwerve.DriveSubsystemSwerve;

/**
 * Change robot drive to slow mode.
 */
public class SetSlowMode extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystemSwerve robotDrive;

  public SetSlowMode(DriveSubsystemSwerve subsystem) {
    robotDrive = subsystem;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    robotDrive.setSpeedMode(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}