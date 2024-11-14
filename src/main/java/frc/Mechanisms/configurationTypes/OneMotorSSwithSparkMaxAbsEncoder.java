package frc.Mechanisms.configurationTypes;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OneMotorSSwithSparkMaxAbsEncoder extends SubsystemBase{
    // motors
    private final CANSparkMax motor;
    private final RelativeEncoder motorEncoder;
    private final SparkPIDController motorPID;
    private final AbsoluteEncoder motorAbsEncoder;
    private final PIDController seekController;

    private double maxRPM = 5700;
    private double kP = 6e-5, kI = 5e-9, kD = 1e-4;
    private double kIz = 0, kFF = 1.0/maxRPM, kMaxOut = 1, kMinOut = -1;
    private double[] defPIDConst = {kP, kI, kD, kIz, kFF, kMinOut, kMaxOut};
    private double speed, velocity;
    private double angleOffset;
    private double[] limit = {0, 180};

    private double seekP, seekI, seekD, seekSpeed, setpoint;

    private String sbName;
    private ShuffleboardTab sbTab;
    private ShuffleboardTab sbCompTab;
    private GenericEntry sbSpeed, sbVelocity, sbAbsAngle, sbAbsReading, sbSetpoint;

    // // DEFAULT PID constants
    // kP = 6e-5;
    // kI = 5e-9;
    // kD = 1e-4;
    // kIz = 0;
    // kFF = 1.0/5700.0;
    // kMaxOut = 1;
    // kMinOut = -1;
    // maxRPM = 5700;

    public OneMotorSSwithSparkMaxAbsEncoder(int motorCanId, boolean inverted, double velConvFactor, String name, double P, double I, double D){
        motor = new CANSparkMax(motorCanId, MotorType.kBrushless);
        motor.setInverted(inverted);
        motorEncoder = motor.getEncoder();
        motorAbsEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        motorEncoder.setVelocityConversionFactor(velConvFactor);
        motorPID = motor.getPIDController();
        setSeekPIDConstants(P, I, D);
        seekController = new PIDController(seekP, seekI, seekD);
        // motorEncoder.setPosition(0);
        setPIDconsts(defPIDConst);
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

    public void runVelocity(){
        motorPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocity(double velocity) {
      this.velocity = velocity;

    }

    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return speed;
    }

    public void run(){
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
        setVelocity(0);
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

    public void setSeekPIDConstants(double P, double I, double D){
      seekP = P;
      seekI = I;
      seekD = D;
    }

    public void move(double speed) {
        // Check limits when moving down
        if (inRange(speed)) {
          motor.set(speed);
        } else {
          motor.set(0);
        }
      }
    
      private boolean moveIn(double speed) {
        if (speed < 0) {
          return true;
        } else {
          return false;
        }
      }

      public double getRefAngle() {
        return ((getRawAngle() + angleOffset) % 360);
      }
    
      private boolean inRange(double speed) {
        if(moveIn(speed) && (getRefAngle() > limit[0])) {
          return true;
        } else if (!moveIn(speed) && (getRefAngle() < limit[1])) {
          return true;
        } else {
          return false;
        }
      }
    
      private double getRawAngle() {
        return motorAbsEncoder.getPosition() * 360.0;
      }

      public void setAbsAngleOffset(double offset) {
        angleOffset = offset;
      }

      public double getAbsPosition(){
        return motorAbsEncoder.getPosition();
      }

      public void setLimits(double upLimit, double downLimit) {
        limit[0] = upLimit;
        limit[1] = downLimit;
      }

      public void seekPosition(double position){
        seekController.setSetpoint(position);
        this.speed = -seekController.calculate(getAbsPosition());
        this.run();
        this.setpoint = position;
      }

      public void seekPositionLimited(double position, double MaxPower, double MinPower, double UpperLimit, double LowerLimit){
        seekController.setSetpoint(position);
        this.speed = seekController.calculate(getAbsPosition());
        this.setpoint = position;
        if(this.speed>MaxPower){
          this.speed = MaxPower;
        } else if(this.speed<MinPower){
          this.speed = MinPower;
        }

        if(getAbsPosition()>UpperLimit && this.speed>0){
          this.speed = 0;
        } else if(getAbsPosition()<LowerLimit && this.speed<0){
          this.speed = 0;
        }
        this.run();
      }

      public void seekPositionLimitedReverseEnc(double position, double MaxPower, double MinPower, double UpperLimit, double LowerLimit){
        seekController.setSetpoint(position);
        this.speed = seekController.calculate(getAbsPosition());
        this.setpoint = position;
        if(this.speed>MaxPower){
          this.speed = MaxPower;
        } else if(this.speed<MinPower){
          this.speed = MinPower;
        }

        if(getAbsPosition()<UpperLimit && this.speed>0){
          this.speed = 0;
        } else if(getAbsPosition()>LowerLimit && this.speed<0){
          this.speed = 0;
        }
        this.run();
      }

      public double getSetpoint(){
        return setpoint;
      }
  // DASHBOARD HELPER FUNCTIONS
    private void dashboardInit() {
        sbTab = Shuffleboard.getTab(sbName);
        sbSpeed = sbTab.add(sbName + " Speed", speed).getEntry();
        sbAbsAngle = sbTab.add(sbName + " Abs Angle", getRawAngle()).getEntry();
        sbSetpoint = sbTab.add(sbName + " Setpoint", setpoint).getEntry();
        sbAbsReading = sbTab.add(sbName + " Abs Enc Reading", getAbsPosition()).getEntry();
    }

    public void dashboardUpdate(){
        sbSpeed.setDouble(speed);
        sbAbsAngle.setDouble(getRawAngle());
        sbSetpoint.setDouble(setpoint);
        sbAbsReading.setDouble(getAbsPosition());
    }
}