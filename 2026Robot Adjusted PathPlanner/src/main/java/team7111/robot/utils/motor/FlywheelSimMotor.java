package team7111.robot.utils.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team7111.robot.utils.encoder.GenericEncoder;

public class FlywheelSimMotor implements Motor{

    private FlywheelSim motor;
    private PIDController pid;
    private GenericEncoder encoder;
    private double velocitySetpoint = 0;


    public FlywheelSimMotor(GenericEncoder encoder, FlywheelSim flywheelSim, PIDController pid, SimpleMotorFeedforward ff){
        motor = flywheelSim;
        this.pid = pid;
        this.encoder = encoder;
    }

    @Override
    public void setDutyCycle(double speed) {
        motor.setInput(speed);
    }

    @Override
    public double getDutyCycle() {
        return motor.getOutput(0);
    }

    @Override
    public void setVelocity(double rpm) {
        //motor.setAngularVelocity(rpm * (2*Math.PI) / 60);
        motor.setInputVoltage(pid.calculate(getVelocity(), rpm));
    }

    @Override
    public double getVelocity() {
        return motor.getAngularVelocityRPM();
    }

    @Override
    public void setPositionReadout(double position) {
        
    }

    @Override
    public double getPosition() {
        return 0;
    }

    @Override
    public void setSetpoint(double setPoint, boolean useFF) {
        
    }

    @Override
    public void setPID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    @Override
    public PIDController getPID() {
        return pid;
    }

    @Override
    public void setSpeedLimits(double positiveSpeed, double negativeSpeed, boolean isVoltage) {
        
    }

    @Override
    public GenericEncoder getEncoder() {
        return encoder;
    }

    @Override
    public double getVoltage() {
        return motor.getInputVoltage();
    }

    @Override
    public double getCurrent() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setInputVoltage(volts);
    }

    @Override
    public boolean isAtSetpoint(double deadzone) {
        return false;
    }

    public boolean isAtVelocitySetpoint(double deadzone){
        return getVelocity() <= velocitySetpoint + deadzone && getVelocity() >= velocitySetpoint - deadzone;
    }

    @Override
    public SimpleMotorFeedforward getFeedForward() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFeedForward'");
    }

    @Override
    public void setFeedFoward(double kS, double kV, double kA) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFeedFoward'");
    }

    @Override
    public void periodic() {
        
    }

}
