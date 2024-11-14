package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubmoduleSubsystemConstants.ConstMode;

public class ssEnableDisable {

    private ShuffleboardTab ssEnableTab;
    private GenericEntry competitionStatus;
    private GenericEntry drivetrainStatus;
    private GenericEntry intakeStatus;
    

    public ssEnableDisable(){
        inCompMode();
        ssEnableTab = Shuffleboard.getTab("Subsystem Status");
        competitionStatus = ssEnableTab.add("Competition", ConstMode.competitionMode).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        drivetrainStatus = ssEnableTab.add("Drivetrain", ConstMode.drivetrainRun).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        intakeStatus = ssEnableTab.add("Intake", ConstMode.intakeRun).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    
    }

    public void inCompMode() {
        if (ConstMode.competitionMode){
            ConstMode.drivetrainRun = true;
            ConstMode.intakeRun = true;
        }
    }

    public void updateSSstatus(){
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
