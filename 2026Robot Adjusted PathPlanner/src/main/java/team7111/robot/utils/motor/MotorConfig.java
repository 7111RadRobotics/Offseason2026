package team7111.robot.utils.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team7111.robot.utils.motor.Motor.MechanismType;

public class MotorConfig {
    public double gearRatio;
    public int currentLimit;
    public boolean isInverted;
    public boolean isBreakMode;
    public PIDController pid;
    public MechanismType mechanism;
    public ArmFeedforward armFF = null;
    public ElevatorFeedforward elevatorFF = null;
    public SimpleMotorFeedforward simpleFF = null;
    public TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    public SparkBaseConfig sparkConfig = new SparkMaxConfig();

    public MotorConfig(
        double gearRatio, int currentLimit, boolean isInverted, boolean isBreakMode, 
        PIDController pid, MechanismType mechanism, double kS, double kV, double kA, double Kg
    ) {
        this.gearRatio = gearRatio;
        this.currentLimit = currentLimit;
        this.isInverted = isInverted;
        this.isBreakMode = isBreakMode;
        this.pid = pid;
        this.mechanism = mechanism;

        simpleFF = new SimpleMotorFeedforward(kS, kV, kA);
        armFF = new ArmFeedforward(kS, Kg, kV, kA);
        elevatorFF = new ElevatorFeedforward(kS, Kg, kV, kA);
        
    }

    public MotorConfig withTalonConfig(TalonFXConfiguration config){
        this.talonConfig = config;
        return this;
    }

    public MotorConfig withSparkConfig(SparkBaseConfig config){
        this.sparkConfig = config;
        return this;
    }

    public MotorConfig withInverted(boolean isInverted){
        this.isInverted = isInverted;
        return this;
    }

    public MotorConfig withCurrentLimit(int limit){
        this.currentLimit = limit;
        return this;
    }
}
