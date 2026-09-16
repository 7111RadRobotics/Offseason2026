package team7111.robot.utils.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveMotorConfig {
    public DCMotor dcMotor;
    public int id;
    public boolean Inverted;
    public boolean isBrakeMode;
    public double gearRatio;
    public double moi;
    public int currentLimit;
    public PIDController pid;
    public SimpleMotorFeedforward ff;

    public SwerveMotorConfig(DCMotor dcMotor, boolean Inverted, boolean isBrakeMode, double gearRatio, double moi, 
            int currentLimit, PIDController pid, SimpleMotorFeedforward ff){
                this.dcMotor = dcMotor;
                this.Inverted = Inverted;
                this.isBrakeMode = isBrakeMode;
                this.gearRatio = gearRatio;
                this.moi = moi;
                this.currentLimit = currentLimit;
                this.pid = pid;
                this.ff = ff;
            
    }
    
    public SwerveMotorConfig(int id, SwerveMotorConfig motorConfig) {
        this(motorConfig.dcMotor, motorConfig.Inverted, motorConfig.isBrakeMode, motorConfig.gearRatio,
             motorConfig.moi, motorConfig.currentLimit, motorConfig.pid, motorConfig.ff);
        this.id = id;
    }
    //hi gidho[ te dpdmrv]
    public SparkMaxConfig getSparkMaxConfig() {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.closedLoopRampRate(0);
        
        //sparkMaxConfig.absoluteEncoder.positionConversionFactor(gearRatio);
        //sparkMaxConfig.encoder.positionConversionFactor(gearRatio);
        
        if (isBrakeMode) {
            sparkMaxConfig.idleMode(IdleMode.kBrake);
        } else {
            sparkMaxConfig.idleMode(IdleMode.kCoast);
        }
            

        sparkMaxConfig.inverted(Inverted)
            .smartCurrentLimit(currentLimit);

        sparkMaxConfig.closedLoop.pid(pid.getP(), pid.getI(), pid.getD())
            .velocityFF(ff.getKv());


        return sparkMaxConfig;
    }

    public TalonFXConfiguration getTalonFXConfiguration() {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        //talonFXConfig.Feedback.SensorToMechanismRatio = gearRatio;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
        talonFXConfig.Slot0.kP = pid.getP();
        talonFXConfig.Slot0.kI = pid.getI();
        talonFXConfig.Slot0.kD = pid.getD();
        talonFXConfig.Slot0.kS = ff.getKs();
        talonFXConfig.Slot0.kV = ff.getKv();
        talonFXConfig.Slot0.kA = ff.getKa();
        talonFXConfig.MotorOutput.Inverted = Inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
        talonFXConfig.MotorOutput.NeutralMode = isBrakeMode
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

        return talonFXConfig;
    }    
}