package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubmoduleSubsystemConstants.ConstMode;

/**
 * Class that allows for enabling and disabling system functions through
 * ShuffleBoard.
 */
public class EnableDisable {

  private ShuffleboardTab ssEnableTab;
  private GenericEntry competitionStatus;
  private GenericEntry drivetrainStatus;
  private GenericEntry intakeStatus;

  /**
   * Initialize ShuffleBoard tab and add buttons for each subsystem.
   */
  public EnableDisable() {
    inCompMode();
    ssEnableTab = Shuffleboard.getTab("Subsystem Status");
    competitionStatus = ssEnableTab.add("Competition Mode", ConstMode.competitionMode).getEntry();
    drivetrainStatus = ssEnableTab.add("Drivetrain Enabled", ConstMode.drivetrainRun).getEntry();
    intakeStatus = ssEnableTab.add("Intake Enabled", ConstMode.intakeRun).getEntry();

  }

  /**
   * Check whether the robot is in competition mode.
   */
  public void inCompMode() {
    if (ConstMode.competitionMode) {
      ConstMode.drivetrainRun = true;
      ConstMode.intakeRun = true;
    }
  }

  /**
   * Updates ShuffleBoard.
   */
  public void updateSubsystemStatus() {
    ConstMode.competitionMode = competitionStatus.getBoolean(false);
    if (ConstMode.competitionMode) {
      ConstMode.drivetrainRun = true;
      ConstMode.intakeRun = true;
    } else {
      ConstMode.drivetrainRun = drivetrainStatus.getBoolean(false);
      ConstMode.intakeRun = intakeStatus.getBoolean(false);
    }

    competitionStatus.setBoolean(ConstMode.competitionMode);
    intakeStatus.setBoolean(ConstMode.intakeRun);
    drivetrainStatus.setBoolean(ConstMode.drivetrainRun);
  }

}
