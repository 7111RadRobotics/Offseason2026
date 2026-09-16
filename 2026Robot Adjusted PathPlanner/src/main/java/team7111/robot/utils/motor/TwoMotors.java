package team7111.robot.utils.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team7111.robot.utils.encoder.GenericEncoder;

public class TwoMotors implements Motor {

    private Motor motor1;
    private Motor motor2;
    private boolean useFollowing = false;

    public TwoMotors(Motor motor1, Motor motor2, int leaderID,  boolean isFollowerInverted){
        this.motor1 = motor1;
        this.motor2 = motor2;
        
        useFollowing = true;
        motor2.setFollower(true, leaderID, isFollowerInverted);
    }

    public TwoMotors(Motor motor1, Motor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
    }
    
    public void setDutyCycle(double speed){
        motor1.setDutyCycle(speed);
        if(!useFollowing)
            motor2.setDutyCycle(speed);
    }

    public double getDutyCycle(){
        return motor1.getDutyCycle();
    }

    public void setVelocity(double rpm){
        motor1.setVelocity(rpm);
        if(!useFollowing)
            motor2.setVelocity(rpm);
    }

    public double getVelocity(){
        return motor1.getVelocity();
    }

    public void setPositionReadout(double position){
        motor1.setPositionReadout(position);
        if(!useFollowing)
            motor2.setPositionReadout(position);
    }
    
    public double getPosition(){
        return motor1.getPosition();
    }
    
    public void setSetpoint(double setPoint, boolean useFF){
        motor1.setSetpoint(setPoint, useFF);
        if(!useFollowing)
            motor2.setSetpoint(setPoint, useFF);
    }

    /** Must be called by the subystems periodic method */
    public void periodic(){
        motor1.periodic();
        motor2.periodic();
    }
    
    public void setPID(double p, double i, double d){
        motor1.setPID(p, i, d);
        motor2.setPID(p, i, d);
    }
    
    public PIDController getPID(){
        return motor1.getPID();
    }

    public GenericEncoder getEncoder(){
        return motor1.getEncoder();
    }

    public double getVoltage(){
        return motor1.getVoltage();
    }

    public double getCurrent(){
        return motor1.getCurrent();
    }

    public void setVoltage(double volts){
        motor1.setVoltage(volts);
        if(!useFollowing)
            motor2.setVoltage(volts);
    }

    public boolean isAtSetpoint(double deadzone){
        return motor1.isAtSetpoint(deadzone);
    }

    public boolean isAtVelocitySetpoint(double deadzone){
        return motor1.isAtVelocitySetpoint(deadzone);
    }

    public SimpleMotorFeedforward getFeedForward(){
        return motor1.getFeedForward();
    }

    public void setFeedFoward(double kS, double kV, double kA){
        motor1.setFeedFoward(kS, kV, kA);
        motor2.setFeedFoward(kS, kV, kA);
    }

    public void setSpeedLimits(double positiveSpeed, double negativeSpeed, boolean isVoltage) {
        motor1.setSpeedLimits(positiveSpeed, negativeSpeed, isVoltage);
        motor2.setSpeedLimits(positiveSpeed, negativeSpeed, isVoltage);
    }
}
