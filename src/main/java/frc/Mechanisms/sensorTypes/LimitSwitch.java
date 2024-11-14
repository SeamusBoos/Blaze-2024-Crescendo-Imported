package frc.Mechanisms.sensorTypes;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitch extends SubsystemBase {
    boolean limitReached;

    private String sbName;
    private ShuffleboardTab sbTab;
    private GenericEntry sbLimitReached;
    private DigitalInput limitSwitch;

    public LimitSwitch(int SwitchPort, String SwitchName){
        limitSwitch = new DigitalInput(SwitchPort);
        sbName = SwitchName;
        dashboardInit();
    }

    @Override
    public void periodic(){
        limitReached = limitSwitch.get();
        dashboardUpdate();
    }

    public boolean getLimitReached(){
        return limitSwitch.get();
    }

    private void dashboardInit() {
        sbTab = Shuffleboard.getTab("Limit Switch");
        sbLimitReached = sbTab.add(sbName + " Limit Reached", limitReached).getEntry();
    }
    private void dashboardUpdate(){
        sbLimitReached.setBoolean(limitReached);
    }
}
