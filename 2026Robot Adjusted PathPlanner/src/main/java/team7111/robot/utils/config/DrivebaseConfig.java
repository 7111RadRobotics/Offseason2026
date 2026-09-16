package team7111.robot.utils.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import team7111.robot.utils.encoder.CTREEncoder;
import team7111.robot.utils.swervemodules.GenericSwerveModule;
import team7111.robot.utils.swervemodules.SimSwerveModule;
import team7111.robot.utils.swervemodules.SparkMaxSwerveModule;
import team7111.robot.utils.swervemodules.TalonFXSwerveModule;
import team7111.robot.utils.swervemodules.CombinedSwerveModule;

public class DrivebaseConfig {
    
    public GenericSwerveModule[] moduleTypes;
    public SwerveModuleConfig[] moduleConstants;
    public double width;
    public double length;
    public double wheelDiameter;
    public SwerveMotorConfig swerveMotorConfig;
    public double gyroOffsetDegrees;

    public static double swervemoduleDistXOffset = Units.inchesToMeters(3); //X distance from the edge of the swerve base to where the swerve module wheel is
    public static double swervemoduleDistYOffset = Units.inchesToMeters(3); //Y distance from the edge of the swerve base to where the swerve module wheel is

    public DrivebaseConfig(
        GenericSwerveModule[] moduleTypes, SwerveModuleConfig[] moduleConstants, 
        double width, double length, double wheelDiameter, double gyroOffsetDegrees
    ){
        this.moduleTypes = moduleTypes;
        this.moduleConstants = moduleConstants;
        this.width = width;
        this.length = length;
        this.wheelDiameter = wheelDiameter;
        this.gyroOffsetDegrees = gyroOffsetDegrees;
    }

    /**
     * Hi simon :)
     */
    public static DrivebaseConfig getSoundWave(boolean isSim){

        double width = Units.inchesToMeters(28);
        double length = Units.inchesToMeters(28);
        double wheelDiameter = Units.inchesToMeters(4);

        double driveGearing = 6.75 / 1.0;
        double angleGearing = 150.0 / 7.0;
        double driveMOI = 0.019835507; //weight of robot in pounds 56.1
        double angleMOI = 0.0000000001;
        int driveCurrentLimit = 40;
        int angleCurrentLimit = 40;
        boolean driveInversion = false;
        boolean angleInversion = true;
        boolean driveBrakeMode = true;
        boolean angleBrakeMode = false;
        PIDController drivePID = new PIDController(0.01, 0.0, 0.0008);
        PIDController anglePID = new PIDController(0.2, 0.0, 0.0);
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0 /*0.001, 0.0*/);
        SimpleMotorFeedforward angleFF = new SimpleMotorFeedforward(0, 0 /*0.001, 0.0*/);

        SwerveMotorConfig driveMotorConfig = new SwerveMotorConfig(DCMotor.getNEO(1), driveInversion, driveBrakeMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF);
        SwerveMotorConfig angleMotorConfig = new SwerveMotorConfig(DCMotor.getNEO(1), angleInversion, angleBrakeMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        
        double canCoder0Offset = isSim
            ? 0
            : -141.4;
        double canCoder1Offset = isSim
            ? 0
            : 167.17;
        double canCoder2Offset = isSim
            ? 0
            : 177.36;
        double canCoder3Offset = isSim
            ? 0
            : -132.1;

        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{

            //Front left
            new SwerveModuleConfig(
                new SwerveMotorConfig(5, driveMotorConfig), 
                new SwerveMotorConfig(6, angleMotorConfig), 
                new CTREEncoder(1, encoderConfig), canCoder0Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            //Front right
            new SwerveModuleConfig(
                new SwerveMotorConfig(7, driveMotorConfig), 
                new SwerveMotorConfig(8, angleMotorConfig), 
                new CTREEncoder(3, encoderConfig), canCoder1Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),

            //Back left
            new SwerveModuleConfig(
                new SwerveMotorConfig(3, driveMotorConfig), 
                new SwerveMotorConfig(4, angleMotorConfig), 
                new CTREEncoder(0, encoderConfig), canCoder2Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            //Back right
            new SwerveModuleConfig(
                new SwerveMotorConfig(1, driveMotorConfig), 
                new SwerveMotorConfig(2, angleMotorConfig), 
                new CTREEncoder(2, encoderConfig), canCoder3Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),
        };

        GenericSwerveModule[] moduleTypes;
        if(isSim){
            drivePID.setPID(1, 0, 0);
            anglePID.setPID(50, 0, 0);
            moduleTypes = new GenericSwerveModule[]{
                new SimSwerveModule(moduleConstants[0]),
                new SimSwerveModule(moduleConstants[1]),
                new SimSwerveModule(moduleConstants[2]),
                new SimSwerveModule(moduleConstants[3]),
            };
        }else{
            moduleTypes = new GenericSwerveModule[]{
                new SparkMaxSwerveModule(moduleConstants[0]),
                new SparkMaxSwerveModule(moduleConstants[1]),
                new SparkMaxSwerveModule(moduleConstants[2]),
                new SparkMaxSwerveModule(moduleConstants[3]),
            };
        }
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, 90);
    }
    public static DrivebaseConfig getStormSurge(boolean isSim){
        double width = Units.inchesToMeters(21.25);
        double length = Units.inchesToMeters(23.25);
        double wheelDiameter = Units.inchesToMeters(3.75);

        double driveGearing = 6.72 / 1.0;
        double angleGearing = 468.0 / 35.0;
        double driveMOI = 0.25;
        double angleMOI = 0.001;
        int driveCurrentLimit = 80;
        int angleCurrentLimit = 40;
        boolean driveInversion = false;
        boolean angleInversion = true;
        boolean driveBrakeMode = true;
        boolean angleBrakeMode = false;
        PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
        PIDController anglePID = new PIDController(50, 0.0, 0.0);
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.001, 0.0);
        SimpleMotorFeedforward angleFF = new SimpleMotorFeedforward(0.001, 0.0);
        SwerveMotorConfig driveMotorConfig = new SwerveMotorConfig(DCMotor.getKrakenX60(1), driveInversion, driveBrakeMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF);
        SwerveMotorConfig angleMotorConfig = new SwerveMotorConfig(DCMotor.getKrakenX60(1), angleInversion, angleBrakeMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5));

        double canCoder0Offset = isSim
            ? 0
            : 14.33;
        double canCoder1Offset = isSim
            ? 0
            : -57.83;
        double canCoder2Offset = isSim
            ? 0
            : -179.73;
        double canCoder3Offset = isSim
            ? 0
            : -140.09;

        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{

            //Front left
            new SwerveModuleConfig(
                new SwerveMotorConfig(11, driveMotorConfig), 
                new SwerveMotorConfig(4, angleMotorConfig), 
                new CTREEncoder(1, encoderConfig), canCoder1Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            //Front right
            new SwerveModuleConfig(
                new SwerveMotorConfig(7, driveMotorConfig), 
                new SwerveMotorConfig(6, angleMotorConfig), 
                new CTREEncoder(2, encoderConfig), canCoder2Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),
            
            //Back left
            new SwerveModuleConfig(
                new SwerveMotorConfig(9, driveMotorConfig), 
                new SwerveMotorConfig(10, angleMotorConfig), 
                new CTREEncoder(3, encoderConfig), canCoder3Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            //Back right
            new SwerveModuleConfig(
                new SwerveMotorConfig(12, driveMotorConfig),
                new SwerveMotorConfig(3, angleMotorConfig),
                new CTREEncoder(0, encoderConfig), canCoder0Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),
        };


        GenericSwerveModule[] moduleTypes;
        if(isSim){
            drivePID.setPID(1, 0, 0);
            anglePID.setPID(50, 0, 0);
            moduleTypes = new GenericSwerveModule[]{
                new SimSwerveModule(moduleConstants[0]),
                new SimSwerveModule(moduleConstants[1]),
                new SimSwerveModule(moduleConstants[2]),
                new SimSwerveModule(moduleConstants[3]),
            };
        }else{
            moduleTypes = new GenericSwerveModule[]{
                new TalonFXSwerveModule(moduleConstants[0]),
                new TalonFXSwerveModule(moduleConstants[1]),
                new TalonFXSwerveModule(moduleConstants[2]),
                new TalonFXSwerveModule(moduleConstants[3])
            };
        }
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, 0);
    }
    
    public static DrivebaseConfig getWhiplash(boolean isSim) {
        double width = Units.inchesToMeters(20);
        double length = Units.inchesToMeters(20);
        double wheelDiameter = Units.inchesToMeters(4.25);

        double driveGearing = 6.72 / 1.0; 
        double angleGearing = 468.0 / 35.0;
        double driveMOI = 0.25;
        double angleMOI = 0.001;
        int driveCurrentLimit = 40;
        int angleCurrentLimit = 40;
        boolean driveInversion = false;
        boolean angleInversion = true;
        boolean driveBrakeMode = true;
        boolean angleBrakeMode = false;
        PIDController drivePID = new PIDController(0.35, 0.001, 0.0);
        PIDController anglePID = new PIDController(0.5, 0.0, 0.0);
        SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.001, 0.0);
        SimpleMotorFeedforward angleFF = new SimpleMotorFeedforward(0.001, 0.0);
        SwerveMotorConfig driveMotorConfig = new SwerveMotorConfig(DCMotor.getKrakenX60(1), driveInversion, driveBrakeMode, driveGearing, driveMOI, driveCurrentLimit, drivePID, driveFF);
        SwerveMotorConfig angleMotorConfig = new SwerveMotorConfig(DCMotor.getNEO(1), angleInversion, angleBrakeMode, angleGearing, angleMOI, angleCurrentLimit, anglePID, angleFF);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.5));

        double canCoder1Offset = isSim
            ? 0
            : -49.92;
        double canCoder2Offset = isSim
            ? 0
            : 101.9;
        double canCoder3Offset = isSim
            ? 0
            : 35.06;
        double canCoder4Offset = isSim
            ? 0
            : -96.15;

        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{

            //Front left
            new SwerveModuleConfig(
                new SwerveMotorConfig(2, driveMotorConfig), 
                new SwerveMotorConfig(2, angleMotorConfig), 
                new CTREEncoder(2, encoderConfig), canCoder2Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            // Front right
            new SwerveModuleConfig(
                new SwerveMotorConfig(4, driveMotorConfig),
                new SwerveMotorConfig(4, angleMotorConfig),
                new CTREEncoder(4, encoderConfig), canCoder4Offset, wheelDiameter * Math.PI,
                new Translation2d(length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),
            
            // Back left
            new SwerveModuleConfig(
                new SwerveMotorConfig(1, driveMotorConfig), 
                new SwerveMotorConfig(1, angleMotorConfig), 
                new CTREEncoder(1, encoderConfig), canCoder1Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, -width/2 - swervemoduleDistYOffset)),

            //Back right
            new SwerveModuleConfig(
                new SwerveMotorConfig(3, driveMotorConfig), 
                new SwerveMotorConfig(3, angleMotorConfig), 
                new CTREEncoder(3, encoderConfig), canCoder3Offset, wheelDiameter * Math.PI,
                new Translation2d(-length/2 - swervemoduleDistXOffset, width/2 - swervemoduleDistYOffset)),
        };

        GenericSwerveModule[] moduleTypes;
        if(isSim){
            drivePID.setPID(1, 0, 0);
            anglePID.setPID(50, 0, 0);
            moduleTypes = new GenericSwerveModule[]{
                new SimSwerveModule(moduleConstants[0]),
                new SimSwerveModule(moduleConstants[1]),
                new SimSwerveModule(moduleConstants[2]),
                new SimSwerveModule(moduleConstants[3]),
            };
        }else{
            moduleTypes = new GenericSwerveModule[]{
                new CombinedSwerveModule(
                    new TalonFXSwerveModule(moduleConstants[0], true), 
                    new SparkMaxSwerveModule(moduleConstants[0], false), 
                    moduleConstants[0]),
                new CombinedSwerveModule(
                    new TalonFXSwerveModule(moduleConstants[1], true), 
                    new SparkMaxSwerveModule(moduleConstants[1], false), 
                    moduleConstants[1]),
                new CombinedSwerveModule(
                    new TalonFXSwerveModule(moduleConstants[2], true), 
                    new SparkMaxSwerveModule(moduleConstants[2], false), 
                    moduleConstants[2]),
                new CombinedSwerveModule(
                    new TalonFXSwerveModule(moduleConstants[3], true), 
                    new SparkMaxSwerveModule(moduleConstants[3], false), 
                    moduleConstants[3]),
            };
        }
        
        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, 0);
    }

    public static DrivebaseConfig getBoxChassis(){
        GenericSwerveModule[] moduleTypes = new GenericSwerveModule[]{
            
        };
        SwerveModuleConfig[] moduleConstants = new SwerveModuleConfig[]{

        };
        double width = 0;
        double length = 0;
        double wheelDiameter = 4;
        double moi = 0.001;

        return new DrivebaseConfig(moduleTypes, moduleConstants, width, length, wheelDiameter, 0);
    }
}
