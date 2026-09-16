package team7111.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.utils.encoder.GenericEncoder;

public class CTREMotor implements Motor {
    private TalonFX motor;
    private PIDController pid = new PIDController(0.05, 0, 0);
    private TalonFXConfiguration talonConfig;
    private MechanismType mechanismType;
    private GenericEncoder encoder = null;
    private double gearRatio;
    private double currentSetpoint;
    private double velocitySetpoint = 0;
    private double positiveVoltageLimit = 100;
    private double negativeVoltageLimit = -100;
    private double positiveVelocityLimit = 5000;
    private double negativeVelocityLimit = -5000;
    private Follower follower;
    private SimpleMotorFeedforward feedforward;
    private ArmFeedforward armFF;
    private ElevatorFeedforward elevatorFF;
    private int id;
    private boolean isFollower = false;
    private boolean isFollowerInverted = false;
    private int leaderID;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    private GenericEntry motorPEntry;
    private GenericEntry motorIEntry;
    private GenericEntry motorDEntry;
    
    public CTREMotor(int id, GenericEncoder encoder, MotorConfig config){
        this.encoder = encoder;
        this.gearRatio = config.gearRatio;
        this.pid = config.pid;
        this.feedforward = config.simpleFF;
        this.armFF = config.armFF;
        this.elevatorFF = config.elevatorFF;
        this.id = id;
        motor = new TalonFX(id);
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        config.talonConfig.Slot0.kP = pid.getP();
        config.talonConfig.Slot0.kI = pid.getI();
        config.talonConfig.Slot0.kD = pid.getD();
        config.talonConfig.Slot0.kS = feedforward.getKs();
        config.talonConfig.Slot0.kV = feedforward.getKv();
        config.talonConfig.Slot0.kA = feedforward.getKa();
        config.talonConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit;
        config.talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.talonConfig.MotorOutput.Inverted = config.isInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive; 
        config.talonConfig.MotorOutput.NeutralMode = config.isBreakMode
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
        motor.getConfigurator().apply(config.talonConfig);
        setPositionReadout(0);

        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Voltage", () -> motor.getMotorVoltage().getValueAsDouble()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Speed", () -> motor.get()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Position", () -> 360 * motor.getRotorPosition().getValueAsDouble()).withWidget("");
    }

    public CTREMotor(int id){
        this.id = id;
        this.mechanismType = MechanismType.flywheel;
        motor = new TalonFX(id);
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        setPositionReadout(0);
    }

    public void setDutyCycle(double speed){
        motor.set(speed);
    }

    public double getDutyCycle(){
        return motor.get();
    }

    public void setVelocity(double rpm){
        
        if(rpm > positiveVelocityLimit){
            rpm = positiveVelocityLimit;
        }
        if(rpm < negativeVelocityLimit){
            rpm = negativeVelocityLimit;
        }
        velocitySetpoint = rpm * gearRatio;

        rpm /= 60;
        motor.setControl(velocityVoltage.withVelocity(rpm * gearRatio));
    }

    public double getVelocity(){
        return motor.getVelocity().getValueAsDouble() * 60 / gearRatio;
    }
    
    public double getCurrent(){
        
        return motor.getStatorCurrent().getValueAsDouble();
    }
    
    public void setPositionReadout(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        } else {
            motor.setPosition(Degrees.of(position).in(Rotations) * gearRatio);
        }
    }

    
    public double getPosition(){
        if(encoder == null){
            return motor.getPosition().getValueAsDouble() / gearRatio * 360;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }
        
    
    public void setSetpoint(double setPoint, boolean useFF){
        currentSetpoint = setPoint;
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = 0;
        
        if(useFF){
            switch(mechanismType){
                case arm:
                    feedforwardOutput = armFF.calculate(Degrees.of(setPoint).in(Radians), pid.getErrorDerivative());
                    break;
                case elevator:
                    feedforwardOutput = elevatorFF.calculate(pid.getErrorDerivative());
                    break;
                case flywheel:
                default:
                    feedforwardOutput = feedforward.calculate(pid.getErrorDerivative());
                    break;
            }
        }

        double totalOutput = pidOutput + feedforwardOutput;
        SmartDashboard.putNumber("Motor " + id + " pid", totalOutput);
        if(totalOutput > positiveVoltageLimit){
            motor.setVoltage(positiveVoltageLimit);
            //motor.set(positiveVoltageLimit);
        }else if(totalOutput < negativeVoltageLimit){
            motor.setVoltage(negativeVoltageLimit);
            //motor.set(negativeVoltageLimit);
        }else{
            motor.setVoltage(totalOutput);
        }
        
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public PIDController getPID() {
        return pid;
    }

    public GenericEncoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    
    public void setVoltage(double volts){
        motor.setVoltage(volts);
    }
    
    public boolean isAtSetpoint(double deadzone){
        if(getPosition() >= currentSetpoint - deadzone && getPosition() <= currentSetpoint + deadzone){
            return true;
        }
        return false;
    }

    public boolean isAtVelocitySetpoint(double deadzone){
        return getVelocity() >= velocitySetpoint - deadzone && getVelocity() <= velocitySetpoint + deadzone;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return feedforward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kV = kV;
        talonConfig.Slot0.kA = kA;
        motor.getConfigurator().apply(talonConfig);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void setFollower(boolean isFollower, int id, boolean isInverted){
        this.isFollower = isFollower;
        this.leaderID = id;
        this.isFollowerInverted = isInverted;
        MotorAlignmentValue motorAlignmentValue = isFollowerInverted
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
        follower = new Follower(id, motorAlignmentValue);
    }

    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
        SmartDashboard.putNumber("Motor " + id + " setpoint", currentSetpoint);
        SmartDashboard.putBoolean("Motor " + id + " isAtSetpoint", isAtSetpoint(0.05));

        MotorAlignmentValue motorAlignmentValue = isFollowerInverted
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;

        if (isFollower) {
            motor.setControl(follower.withLeaderID(leaderID).withMotorAlignment(motorAlignmentValue));
        }
        /*pid = new PIDController(
            motorPEntry.getDouble(0), 
            motorIEntry.getDouble(0), 
            motorDEntry.getDouble(0));*/
    }

    public void setSpeedLimits(double positiveSpeed, double negativeSpeed, boolean isVoltage) {
        if(isVoltage){
            positiveVoltageLimit = positiveSpeed;
            negativeVoltageLimit = negativeSpeed;
        }else{
            positiveVelocityLimit = positiveSpeed;
            negativeVelocityLimit = negativeSpeed;
        }
    }
}
