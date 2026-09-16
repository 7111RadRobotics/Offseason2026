package team7111.robot.utils.motor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import team7111.robot.utils.encoder.GenericEncoder;

public class ElevatorSimMotor implements Motor{
    private ElevatorSim motor; 
    private PIDController pid;
    private GenericEncoder encoder;
    private double gearRatio;
    private double setPoint;
    private ElevatorFeedforward feedForward;
    private Motor simType;

    private double velocityMPS = 0;
    private double positionMeters = 0;
    private double outputVoltage = 0;
    private double outputCurrent = 0;

    public ElevatorSimMotor(GenericEncoder encoder, double gearRatio, PIDController pid, ElevatorFeedforward feedForward, ElevatorSim elevatorSim){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedForward;
        this.motor = elevatorSim;       
    }

    public void setDutyCycle(double speed){
        motor.setInput(speed);
    }

    public double getDutyCycle(){
        return motor.getOutput(0);
    }

    public void setPositionReadout(double position){
        motor.setState(position, 0);
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getPositionMeters();
        } else{
            return encoder.getPosition().getDegrees();
        }
    }        
    
    public void setSetpoint(double setPoint, boolean useFF){
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = useFF && feedForward != null 
            ? feedForward.calculate(pid.getErrorDerivative())
            : 0;

        outputVoltage = pidOutput + feedforwardOutput;

        motor.setInputVoltage(outputVoltage);
        this.setPoint = setPoint;
    }
    
    public void periodic(){
        if (encoder != null){
            encoder.periodic();
            encoder.setPosition(Rotation2d.fromDegrees(positionMeters));
        }
        motor.update(0.020);
        //motor.setState(positionMeters, velocityMPS);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(motor.getCurrentDrawAmps()));
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
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

    public double getCurrent(){
        return outputCurrent;
    }
    public void setVoltage(double volts) {
        motor.setInputVoltage(volts);
    }
    
    public boolean isAtSetpoint(double deadzone){
        if (getPosition() >= setPoint - deadzone && getPosition() <= setPoint + deadzone)
            return true;
        else 
            return false;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return new SimpleMotorFeedforward(feedForward.getKs(), feedForward.getKv(), feedForward.getKa());
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedForward = new ElevatorFeedforward(kS, feedForward.getKg(), kV, kA);
    }

    @Override
    public void setSpeedLimits(double positiveSpeed, double negativeSpeed, boolean isVoltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeedLimits'");
    }

    @Override
    public void setVelocity(double rpm) {
        //does nothing
    }

    @Override
    public double getVelocity() {
        return motor.getVelocityMetersPerSecond() / gearRatio;
    }

    @Override
    public boolean isAtVelocitySetpoint(double deadZone) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isAtVelocitySetpoint'");
    }
}
