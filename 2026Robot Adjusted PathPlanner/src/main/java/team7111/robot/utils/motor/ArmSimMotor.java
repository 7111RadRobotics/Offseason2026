package team7111.robot.utils.motor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team7111.robot.utils.encoder.GenericEncoder;

public class ArmSimMotor implements Motor {

    private SingleJointedArmSim motor;
    private GenericEncoder encoder;
    private PIDController pid;
    private ArmFeedforward feedforward;
    private double setpoint = 0;
    private double outputVoltage = 0;
    private double outputCurrent = 0;
    
    public ArmSimMotor(GenericEncoder encoder, SingleJointedArmSim armSim, PIDController pid, ArmFeedforward feedforward){
        motor = armSim;
        this.encoder = encoder;
        this.pid = pid;
        this.feedforward = feedforward;
    }

    public void setDutyCycle(double speed){
        motor.setInput(speed);
    }

    public double getDutyCycle(){
        return motor.getOutput(0);
    }

    public void setPositionReadout(double position){
        
    }
    
    public double getPosition(){
        if(encoder != null){
            return encoder.getPosition().getDegrees();
        }
        return Units.radiansToDegrees(motor.getAngleRads());
    }
    
    public void setSetpoint(double setPoint, boolean useFF){

        double useFeedForward = 0;
        if(useFF){
            if (feedforward != null) {
                useFeedForward = feedforward.calculate(getPosition(), pid.getErrorDerivative());
            }
        }
        outputVoltage = pid.calculate(getPosition(), setPoint) + useFeedForward;
        this.setpoint = setPoint;
        motor.setInputVoltage(outputVoltage);
        
        
    }

    /** Must be called by the subystems periodic method */
    public void periodic(){
        motor.update(0.02);
    }
    
    public void setPID(double p, double i, double d){
        pid = new PIDController(p, i, d);
    }
    
    public PIDController getPID(){
        return pid;
    }

    public GenericEncoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return outputVoltage;
    }
    
    public double getCurrent() {
        return outputCurrent;
    }

    public void setVoltage(double volts){
        motor.setInputVoltage(volts);
    }

    public boolean isAtSetpoint(double deadzone){
        return getPosition() <= setpoint + deadzone && getPosition() >= setpoint - deadzone;
    }

    public boolean isAtVelocitySetpoint(double deadzone){
        return false;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return new SimpleMotorFeedforward(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedforward = new ArmFeedforward(kS, feedforward.getKg(), kV, kA);
    }

    @Override
    public void setVelocity(double rpm) {
        // does nothing
    }

    @Override
    public double getVelocity() {
        return motor.getVelocityRadPerSec() / (2*Math.PI);
    }
}
