package frc.Mechanisms.sensorTypes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
    String sbName;
    NetworkTable LimelightTable;
    double x, y, v, area, skew, id_num, pipeNum, ledNum, camX, camY, camZ, camRY, priorityIdNum;
    double LastKnownPitch = 0;
    NetworkTableEntry tx, ty, ta, ts, tid, tv, ledMode, pipeline, priorityid;
    NetworkTableEntry cameraSpaceX, cameraSpaceY, cameraSpaceZ, cameraSpaceRY;
    public static double[] botposeblue;
    public static double[] targetPoseCameraSpace = new double[6];
    private ShuffleboardTab sbTab;
    private GenericEntry limelightID, limelightX, limelightY, limelightV, limelightArea, limelightSkew, limelightPipeline, PredictedPitch, realIn;
    private GenericEntry LLCameraSpaceX, LLCameraSpaceY, LLCameraSpaceZ, LLCamSpaceXMeters, LLCameraSpaceRY;
    public Limelight (String TableName, String SbTabName) {
        LimelightTable = NetworkTableInstance.getDefault().getTable(TableName);
        sbName = SbTabName;
        updateLimelight();
        dashboardInit();
    }

    @Override
    public void periodic() {
        updateLimelight();
        dashboardUpdate();

        if(sbName.equals("Shooter Limelight")){
        if(DriverStation.getAlliance().get() == Alliance.Red){
            setPriorityID(4);
        }
        else if(DriverStation.getAlliance().get() == Alliance.Blue){
            setPriorityID(7);
        }
        }
    }

    private void updateLimelight() {
        tid = LimelightTable.getEntry("tid");
        priorityid = LimelightTable.getEntry("priorityid");
        tx = LimelightTable.getEntry("tx");
        ty = LimelightTable.getEntry("ty");
        // ta = LimelightTable.getEntry("ta");
        // ts = LimelightTable.getEntry("ts");
        tv = LimelightTable.getEntry("tv");
        ledMode = LimelightTable.getEntry("ledMode");
        pipeline = LimelightTable.getEntry("pipeline");
        // botposeblue = LimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        targetPoseCameraSpace = LimelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]).clone();
        

        id_num = tid.getDouble(0.0);
        priorityIdNum = priorityid.getInteger(0);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        v = tv.getDouble(0.0);
        // area = ta.getDouble(0.0);
        // skew = ts.getDouble(0.0);
        ledNum = ledMode.getDouble(0);
        pipeNum = pipeline.getInteger(0);
        if(targetPoseCameraSpace.length > 0) {
            camX = targetPoseCameraSpace[0];
            camY = targetPoseCameraSpace[1];
            camZ = targetPoseCameraSpace[2];
            camRY = targetPoseCameraSpace[4];
        }
    }

    public void setPriorityID(int id) {
        LimelightTable.getEntry("priorityid").setInteger(id);
    }

    public void setPipeline(int pl){
        LimelightTable.getEntry("pipeline").setNumber(pl);
    }

    public void setLedMode(int mode){
        LimelightTable.getEntry("ledMode").setNumber(mode);
    }

    public double getV(){
        return v;
    }
    
    public double getX(){
        updateLimelight();
        return x;
    }

    public double getY(){
        updateLimelight();
        return y;
    }

    public double getCamXMeters(){
        updateLimelight();
        return camX;
    }

    public double getCamYMeters(){
        updateLimelight();
        return camY;
    }

    public double getCamZMeters(){
        updateLimelight();
        return camZ;
    }

    public double getCamXInches(){
        updateLimelight();
        return camX*39.37;
    }

    public double getCamYInches(){
        updateLimelight();
        return camY*39.37;
    }

    public double getCamZInches(){
        updateLimelight();
        return camZ*39.37;
    }

    public double getCamRY(){
        return camRY;
    }

    public double getS(){
        updateLimelight();
        return skew;
    }

    public double getRealDistanceInches(){
        double a = -5.36;
        double b = 1.01*(getCamZInches());
        double c = Math.pow(1.05E-04*(getCamZInches()), 2);

        if(Math.abs(getCamRY())>10 && getCamZInches()>100){
            return (a+b+c)-10;
        } else {
            return a+b+c;
    }
    }

    public double getPredictedPivot(){
        double a = -0.0121;
        double b = 7.52E-03*getRealDistanceInches();
        double c = -4.3E-05 * Math.pow(getRealDistanceInches(), 2);
        double d = 8.6E-08 * Math.pow(getRealDistanceInches(), 3);

        if(getV()==1){
        LastKnownPitch = a+b+c+d;
        return a+b+c+d;
        } else {
        return LastKnownPitch;
        }
    }

    private void dashboardInit() {
        sbTab = Shuffleboard.getTab(sbName);
        limelightID = sbTab.add("TagID", id_num).getEntry();
        limelightX = sbTab.add("X", x).getEntry();
        limelightY = sbTab.add("Y", y).getEntry();
        limelightV = sbTab.add("V", v).getEntry();
        // limelightArea = sbTab.add("Area", area).getEntry();
        // limelightSkew = sbTab.add("Skew", skew).getEntry();
        limelightPipeline = sbTab.add("Pipeline", pipeNum).getEntry();
        PredictedPitch = sbTab.add("Predicted Pitch", getPredictedPivot()).getEntry();
        LLCameraSpaceX = sbTab.add("CamXInches", getCamXInches()).getEntry();
        LLCameraSpaceY = sbTab.add("CamYInches", getCamYInches()).getEntry();
        LLCameraSpaceZ = sbTab.add("CamZInches", getCamZInches()).getEntry();
        LLCamSpaceXMeters = sbTab.add("CamXMeters", getCamXMeters()).getEntry();
        LLCameraSpaceRY = sbTab.add("CamRY", getCamRY()).getEntry();
        realIn = sbTab.add("Real Inches Calculated", getRealDistanceInches()).getEntry();
    }

    private void dashboardUpdate(){
        limelightID.setDouble(id_num);
        limelightX.setDouble(x);
        limelightY.setDouble(y);
        limelightV.setDouble(v);
        // limelightArea.setDouble(area);
        // limelightSkew.setDouble(skew);
        limelightPipeline.setDouble(pipeNum);
        PredictedPitch.setDouble(getPredictedPivot());
        LLCameraSpaceX.setDouble(getCamXInches());
        LLCameraSpaceY.setDouble(getCamYInches());
        LLCameraSpaceZ.setDouble(getCamZInches());
        LLCamSpaceXMeters.setDouble(getCamXMeters());
        LLCameraSpaceRY.setDouble(getCamRY());
        realIn.setDouble(getRealDistanceInches());
    }

    public static final class alignmentConstants{
        public static final double Aligned = 0; // slight bias 2 degrees to the right
        public static final double AlignedSpeaker = -0.15;
        public static final double SpeakerTolerance = 0.2;

    }
}
