package frc.Mechanisms.configurationTypes;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwoMotorSSwithSparkMax extends SubsystemBase{
    // motors
    private CANSparkMax motor1, motor2;
    private RelativeEncoder motorEncoder1, motorEncoder2;
    private SparkPIDController motorPID1, motorPID2;
    private PIDController positionController;

    private double maxRPM = 5700;
    // PID (kP, kI, kD, kIZ, kFF, kMaxOut, kMinOut)
    private double[] pidVals1 = {6e-15, 0, 0, 0, 1.0/maxRPM, -1, 1};
    private double[] pidVals2 = {6e-15, 0, 0, 0, 1.0/maxRPM, -1, 1};
    private double[][] pidVals = {pidVals1, pidVals2};
        
    private double speed, velocity, seekSpeed, position;
    private int canId1, canId2;

    private String sbName;
    private ShuffleboardTab sbTab;
    // private ShuffleboardTab sbCompTab;
    private GenericEntry sbSpeed, sbVelocity, sbMeasuredVel1, sbMeasuredVel2, sbPosition;
    private GenericEntry sbCanId1, sbCanId2, sbAtSpeed;

    // // DEFAULT PID constants
    // kP = 6e-5;
    // kI = 5e-9;
    // kD = 1e-4;
    // kIz = 0;
    // kFF = 1.0/5700.0;
    // kMaxOut = 1;
    // kMinOut = -1;
    // maxRPM = 5700;

    public TwoMotorSSwithSparkMax(int motorCanId1, int motorCanId2, boolean inverted1, boolean inverted2, double velConvFactor1, double velConvFactor2, String name){
        motor1 = new CANSparkMax(motorCanId1, MotorType.kBrushless);
        motor2 = new CANSparkMax(motorCanId2, MotorType.kBrushless);
        motor1.setInverted(inverted1);
        motor2.setInverted(inverted2);
        motorEncoder1 = motor1.getEncoder();
        motorEncoder2 = motor2.getEncoder();
        motorEncoder1.setVelocityConversionFactor(velConvFactor1);
        motorEncoder2.setVelocityConversionFactor(velConvFactor2);
        motorPID1 = motor1.getPIDController();
        motorPID2 = motor2.getPIDController();
        canId1 = motorCanId1;
        canId2 = motorCanId2;
        sbName = name;
        resetEncoders();
        setPIDconsts(pidVals);
        dashboardInit();
        positionController = new PIDController(2e-2, 5e-2, 0);
    }

    @Override
    public void periodic() {
        dashboardUpdate();
        updatePosition();
    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoders(){
        motorEncoder1.setPosition(0);
        motorEncoder2.setPosition(0);
    }

    public void runVelocity(){
        motorPID1.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        motorPID2.setReference(-velocity, CANSparkMax.ControlType.kVelocity);
    }

    public double[] getVelocity(){
        double[] motorVels = {motorEncoder1.getVelocity(), motorEncoder2.getVelocity()};
        return motorVels;
    }

    public void setTargetVelocity(double vel) {
        velocity = vel;
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return speed;
    }

    public void run(){
        motor1.set(speed);
        motor2.set(speed);
    }

    public void runMotor1Seperate(double inputSpeed){
        motor1.set(inputSpeed);
    }

    public void runMotor2Seperate(double inputSpeed){
        motor2.set(inputSpeed);
    }

    public void runMotor1(){
        motor1.set(speed);
    }

    public void runMotor2(){
        motor2.set(speed);
    }

    public void stopMotor1(){
        motor1.set(0);
    }

    public void stopMotor2(){
        motor2.set(0);
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
        setTargetVelocity(0);
        // runVelocity(); //Commented because running for velocity after stopping wheels will start them again and cause issues as previously seen with shooter.
    }

    public boolean atSpeed() {
        return (Math.abs(motorEncoder1.getVelocity()) >= Math.abs(velocity) * 0.75 && Math.abs(motorEncoder2.getVelocity()) >= Math.abs(velocity) * 0.75);
    }

    public double[][] getPIDconsts(){
        double[][] pidVals = {pidVals1, pidVals2};
        return pidVals;
    }

    public void setPIDconsts(double[][] pidVals) {
        motorPID1.setP(pidVals[0][0]);
        motorPID1.setI(pidVals[0][1]);
        motorPID1.setD(pidVals[0][2]);
        motorPID1.setIZone(pidVals[0][3]);
        motorPID1.setFF(pidVals[0][4]);
        motorPID1.setOutputRange(pidVals[0][5], pidVals[0][6]);

        motorPID2.setP(pidVals[1][0]);
        motorPID2.setI(pidVals[1][1]);
        motorPID2.setD(pidVals[1][2]);
        motorPID2.setIZone(pidVals[1][3]);
        motorPID2.setFF(pidVals[1][4]);
        motorPID2.setOutputRange(pidVals[1][5], pidVals[1][6]);
    }

    public void updatePosition(){
        position = motorEncoder1.getPosition();
    }

    public double getPosition(){
        return position;
    }

    public void seekPosition(double goalPos){
        positionController.setSetpoint(goalPos);
        seekSpeed = positionController.calculate(position);
        motor1.set(seekSpeed);
        motor2.set(seekSpeed);
    }

    // DASHBOARD HELPER FUNCTIONS
    private void dashboardInit() {
        sbTab = Shuffleboard.getTab(sbName);
        sbCanId1 = sbTab.add("CAN ID1", canId1).getEntry();
        sbCanId2= sbTab.add("CAN ID2", canId2).getEntry();
        sbSpeed = sbTab.add("Speed", speed).getEntry();
        sbVelocity = sbTab.add("Target Velocity", velocity).getEntry();
        sbMeasuredVel1 = sbTab.add("Actual Velocity (motor1)", motorEncoder1.getVelocity()).getEntry();
        sbMeasuredVel2 = sbTab.add("Actual Velocity (motor2)", motorEncoder2.getVelocity()).getEntry();
        sbAtSpeed = sbTab.add("At Speed", atSpeed()).getEntry();
        sbPosition = sbTab.add("Position", getPosition()).getEntry();
    }

    public void dashboardUpdate(){
        sbCanId1.setInteger(canId1);
        sbCanId2.setInteger(canId2);
        sbSpeed.setDouble(speed);
        sbVelocity.setDouble(velocity);
        sbMeasuredVel1.setDouble(motorEncoder1.getVelocity());
        sbMeasuredVel2.setDouble(motorEncoder2.getVelocity());
        sbAtSpeed.setBoolean(atSpeed());
        sbPosition.setDouble(position);
    }
    
}