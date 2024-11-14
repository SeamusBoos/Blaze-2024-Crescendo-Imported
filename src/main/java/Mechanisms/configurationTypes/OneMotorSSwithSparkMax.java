package frc.Mechanisms.configurationTypes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubmoduleSubsystemConstants.ConstMaxSwerveDrive.DriveConstants;
import frc.robot.SubmoduleSubsystemConstants.ConstMode;
import frc.robot.SubmoduleSubsystemConstants.ConstProperties;

public class OneMotorSSwithSparkMax extends SubsystemBase{
    // motors
    private final CANSparkMax motor;
    private final RelativeEncoder motorEncoder;
    private final SparkPIDController motorPID;

    private double maxRPM = 5700;
    private double kP = 6e-5, kI = 0, kD = 0, kIz = 0, kFF = 1.0/maxRPM, kMaxOut = 1, kMinOut = -1;
    private double[] defPIDConst = {kP, kI, kD, kIz, kFF, kMinOut, kMaxOut};
    private double speed, velocity;
    private int CanID = 0;

    // Variables for Dashboard
    private String sbName;
    private ShuffleboardTab sbTab;
    private ShuffleboardTab sbCompTab;
    private GenericEntry sbSpeed, sbVelocity, sbCanId;

    // // DEFAULT PID constants
    // kP = 6e-5;
    // kI = 5e-9;
    // kD = 1e-4;
    // kIz = 0;
    // kFF = 1.0/5700.0;
    // kMaxOut = 1;
    // kMinOut = -1;
    // maxRPM = 5700;

    public OneMotorSSwithSparkMax(int motorCanId, boolean inverted, double velConvFactor, String name){
        motor = new CANSparkMax(motorCanId, MotorType.kBrushless);
        CanID = motorCanId;
        motor.setInverted(inverted);
        motorEncoder = motor.getEncoder();
        motorEncoder.setVelocityConversionFactor(velConvFactor);
        motorPID = motor.getPIDController();
        setPIDconsts(defPIDConst);
        motorEncoder.setPosition(0);
        sbName = name;
        dashboardInit();
    }

    @Override
    public void periodic() {
        dashboardUpdate();
    }

    @Override
    public void simulationPeriodic() {

    }

    public void setVelocity(double setpoint){
        velocity = setpoint;
    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    public void runVel(){
        motorPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return speed;
    }

    public void runSpeed(){
        motor.set(speed);
    }

    public void stop() {
        setSpeed(0);
        setVelocity(0);
        motor.set(0);
        motorPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public double[] getPIDconsts(){
        double[] pidVals = {motorPID.getP(), motorPID.getI(), motorPID.getD(), motorPID.getIZone(), motorPID.getFF(), motorPID.getOutputMax(), motorPID.getOutputMin()};
        return pidVals;
    }

    public void setPIDconsts(double[] pidVals) {
        motorPID.setP(pidVals[0]);
        motorPID.setI(pidVals[1]);
        motorPID.setD(pidVals[2]);
        motorPID.setIZone(pidVals[3]);
        motorPID.setFF(pidVals[4]);
        motorPID.setOutputRange(pidVals[5], pidVals[6]);        
    }

    public boolean atSpeed() {
        return motorEncoder.getVelocity() == velocity;
    }

     // DASHBOARD HELPER FUNCTIONS
    private void dashboardInit() {
        sbTab = Shuffleboard.getTab(sbName);
        sbSpeed = sbTab.add("Speed", speed).getEntry();
        sbVelocity = sbTab.add("RPM setpoint", velocity).getEntry();
        sbCanId = sbTab.add("CAN ID", CanID).getEntry();
    }

    public void dashboardUpdate(){
        sbCanId.setInteger(CanID);
        sbSpeed.setDouble(speed);
        sbVelocity.setDouble(velocity);
    }
    
}