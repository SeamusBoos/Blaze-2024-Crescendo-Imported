package frc.Mechanisms.sensorTypes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OpticalDistanceSensor extends SubsystemBase {

    //2024 Note Stuff
    boolean hasNote = false;
    String noteLocation = "No Note";
    //Reg Stuff
    boolean hasSight;
    int sightThreshold;
    int OdsId, ODSReading;
    String sensorName;
    private ShuffleboardTab sbTab;
    private GenericEntry sbOdsId, sbOdsDistanceReading, sbHasSight, sbNoteLocation, sbHasNote;

    private int max_array_size = 2;
    private Deque<Integer> readings = new ArrayDeque<>();

    AnalogInput sensor_ods;

    public OpticalDistanceSensor(int ODSPort, int Threshold, String SensorName) {
        sensor_ods = new AnalogInput(ODSPort);
        OdsId = ODSPort;
        sensorName = SensorName;
        sightThreshold = Threshold;
        updateODS();
        dashboardInit();
    }
    
    @Override
    public void periodic() {
        updateODS();
        getSightStatus();
        dashboardUpdate();
    }

    @Override
    public void simulationPeriodic() {

    }

    private void updateODS(){
        // ODSReading = sensor_ods.getValue(); //Higher Value = Closer
        // if (max_array_size <= readings.size()) {
        //     readings.removeLast();
        // }
        // ODSReading = 0;
        // readings.addFirst(sensor_ods.getValue());
        // readings.forEach((reading) -> ODSReading += reading);
        // ODSReading /= readings.size();

        ODSReading = sensor_ods.getAverageValue();
    }

    public int getReading(){
        updateODS();
        return ODSReading;
    }

    public boolean getSightStatus(){
        if(this.getReading()>sightThreshold){
            hasSight = true;
        } else {
            hasSight = false;
        }
        return hasSight;
    }

    //2024 Note Location
    public void setHasNote(boolean HasNote){
        hasNote = HasNote;
    }
    public void setNoteLocation(String NoteLocation){
        noteLocation = NoteLocation;
    }
    public String getNoteLocation(){
        return noteLocation;
    }


    private void dashboardInit() {
        sbTab = Shuffleboard.getTab("ODS");
        sbOdsId = sbTab.add(sensorName + " ODS Port", OdsId).getEntry();
        sbOdsDistanceReading = sbTab.add(sensorName + " Reading ", ODSReading).getEntry();
        sbHasSight = sbTab.add(sensorName + " Has Sight", hasSight).getEntry();

        //2024 Note Location
        sbHasNote = sbTab.add(sensorName + "Has Note", hasNote).getEntry();
        sbNoteLocation = sbTab.add(sensorName + "Note Location", noteLocation).getEntry();
    }

    public void dashboardUpdate(){
        sbOdsId.setInteger(OdsId);
        sbOdsDistanceReading.setInteger(ODSReading);
        sbHasSight.setBoolean(hasSight);

        //2024 Note Location
        sbHasNote.setBoolean(hasNote);
        sbNoteLocation.setString(noteLocation);
    }
}
