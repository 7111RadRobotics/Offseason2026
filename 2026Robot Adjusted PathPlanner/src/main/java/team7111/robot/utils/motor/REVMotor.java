package team7111.robot.utils.motor;

import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.utils.encoder.GenericEncoder;


public class REVMotor implements Motor {
    private SparkMax motor;
    private SparkBaseConfig sparkConfig;
    private PIDController pid;
    private GenericEncoder encoder = null;
    private double gearRatio = 1;
    private MechanismType mechanismType = null;
    private SimpleMotorFeedforward simpleMotorFeedforward = null;
    private ElevatorFeedforward elevatorFeedforward = null;
    private ArmFeedforward armFeedforward = null;
    private double setPoint;
    private double velocitySetpoint;
    private double positiveVoltageLimit = 100;
    private double negativeVoltageLimit = -100;
    private double positiveVelocityLimit = 5000;
    private double negativeVelocityLimit = -5000;
    private boolean isFollower = false;
    private int id;
    
    public REVMotor (int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        this.id = id;
    }
    
    public REVMotor(int id, GenericEncoder encoder, MotorConfig config){
        this.encoder = encoder;
        this.gearRatio = config.gearRatio;
        this.pid = config.pid;
        this.armFeedforward = config.armFF;
        this.elevatorFeedforward = config.elevatorFF;
        this.simpleMotorFeedforward = config.simpleFF;
        this.sparkConfig = config.sparkConfig;
        this.mechanismType = config.mechanism;
        this.id = id;
    
        motor = new SparkMax(id, MotorType.kBrushless);

        sparkConfig.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
        sparkConfig.closedLoop.feedForward.sva(config.simpleFF.getKs(), config.simpleFF.getKs(), config.simpleFF.getKs(), ClosedLoopSlot.kSlot1);
        sparkConfig.smartCurrentLimit(config.currentLimit);
        sparkConfig.inverted(config.isInverted);
        IdleMode idleMode = config.isBreakMode
            ? IdleMode.kBrake
            : IdleMode.kCoast;
        sparkConfig.idleMode(idleMode);
        motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Voltage", () -> motor.getBusVoltage()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Speed", () -> motor.get()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Position", () -> motor.getAbsoluteEncoder().getPosition()).withWidget("");
    }


    public void setDutyCycle(double speed){
        motor.set(speed);
    }

    public double getDutyCycle(){
        return motor.get();
    }

    public void setVelocity(double rpm){
        velocitySetpoint = rpm;
        if(velocitySetpoint > positiveVelocityLimit){
            velocitySetpoint = positiveVelocityLimit;
        }
        if(velocitySetpoint < negativeVelocityLimit){
            velocitySetpoint = negativeVelocityLimit;
        }
        motor.getClosedLoopController().setSetpoint(rpm / gearRatio, ControlType.kVelocity);
    }

    public double getVelocity(){
        return motor.getEncoder().getVelocity() * gearRatio;
    }
    
    public void setPositionReadout(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        } else {
            motor.getEncoder().setPosition(Degrees.of(position / gearRatio).in(Rotations));
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getEncoder().getPosition() * gearRatio * 360;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }  
    
    public void setSetpoint(double setPoint, boolean useFF){
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = 0;
        if(useFF){
            switch(mechanismType){
                case arm:
                    feedforwardOutput = armFeedforward.calculate(Degrees.of(setPoint).in(Radians), pid.getErrorDerivative());
                    break;
                case elevator:
                    feedforwardOutput = elevatorFeedforward.calculate(pid.getErrorDerivative());
                    break;
                case flywheel:
                default:
                    feedforwardOutput = simpleMotorFeedforward.calculate(pid.getErrorDerivative());
                    break;
            }
        }
        double output = pidOutput + feedforwardOutput;
    
        this.setPoint = setPoint;
        if(output > positiveVoltageLimit){
            output = positiveVoltageLimit;
        }else if(output < negativeVoltageLimit){
            output = negativeVoltageLimit;
        }
        motor.setVoltage(output); //Needs velocity for feedforward
        SmartDashboard.putNumber("Motor " + id + " pid output", output);
    }

    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
    }

    public void setFollower(boolean isFollower, int id, boolean isInverted){
        if(isFollower && !this.isFollower){
            sparkConfig.follow(id, isInverted);
            motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        if(!isFollower && this.isFollower){
            sparkConfig.disableFollowerMode();
            motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        this.isFollower = isFollower;
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public void setVelocityPID(double p, double i, double d){
        sparkConfig.closedLoop.pid(p, i, d);
        motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public PIDController getPID(){
        return pid;
    }

    public GenericEncoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return motor.getBusVoltage();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    public void setVoltage(double volts){
        motor.setVoltage(volts);
    }

    public boolean isAtSetpoint(double deadzone){
        if (getPosition() >= setPoint - deadzone && getPosition() <= setPoint + deadzone)
            return  true;
        else
            return false;
    }

    public boolean isAtVelocitySetpoint(double deadzone){
        return getVelocity() >= velocitySetpoint - deadzone && getVelocity() <= velocitySetpoint + deadzone;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return simpleMotorFeedforward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        simpleMotorFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void setSpeedLimits(double positiveLimit, double negativeLimit, boolean isVoltage){
        if(isVoltage){
            this.positiveVoltageLimit = positiveLimit;
            this.negativeVoltageLimit = negativeLimit;
        }else{
            positiveVelocityLimit = positiveLimit;
            negativeVelocityLimit = negativeLimit;
        }
    }
    
}
