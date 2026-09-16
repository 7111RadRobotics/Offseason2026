package team7111.robot.utils.swervemodules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.Constants;
import team7111.robot.Constants.SwerveConstants;
import team7111.robot.utils.config.SwerveModuleConfig;
import team7111.robot.utils.encoder.GenericEncoder;

public class TalonFXSwerveModule implements GenericSwerveModule{

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private GenericEncoder encoder;

    private TalonFXConfiguration driveConfig;
    private TalonFXConfiguration angleConfig;

    private DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private SimpleMotorFeedforward driveFF;

    private PositionVoltage anglePosition = new PositionVoltage(0);

    private double encoderOffsetDegrees;
    private double driveGearRatio;
    private double angleGearRatio;
    private double wheelCircumference;
    private double driveRotationsToMeters;

    private boolean useDriveMotor = true;
    private boolean useAngleMotor = true;

    Translation2d fromCenter;

    public TalonFXSwerveModule(SwerveModuleConfig config){
        driveMotor = new TalonFX(config.driveMotor.id, Constants.canbus);
        angleMotor = new TalonFX(config.angleMotor.id, Constants.canbus);
        encoder = config.encoder;

        driveFF = new SimpleMotorFeedforward(config.driveMotor.ff.getKs(), config.driveMotor.ff.getKv(), config.driveMotor.ff.getKa());
        driveConfig = config.driveMotor.getTalonFXConfiguration();
        angleConfig = config.angleMotor.getTalonFXConfiguration();
        driveGearRatio = config.driveMotor.gearRatio;
        angleGearRatio = config.angleMotor.gearRatio;
        wheelCircumference = config.wheelCircumference;
        driveRotationsToMeters = wheelCircumference/driveGearRatio;

        encoderOffsetDegrees = config.canCoderOffsetDegrees;

        this.fromCenter = config.fromCenter;
    }

    public TalonFXSwerveModule(SwerveModuleConfig config, boolean isDriveMotor){
        
        if(isDriveMotor) {
            driveMotor = new TalonFX(config.driveMotor.id, Constants.canbus);
            useAngleMotor = false;
        } else {
            angleMotor = new TalonFX(config.angleMotor.id, Constants.canbus);
            useDriveMotor = false;
        }
        
        encoder = config.encoder;

        driveFF = new SimpleMotorFeedforward(config.driveMotor.ff.getKs(), config.driveMotor.ff.getKv(), config.driveMotor.ff.getKa());
        driveConfig = config.driveMotor.getTalonFXConfiguration();
        angleConfig = config.angleMotor.getTalonFXConfiguration();
        driveGearRatio = config.driveMotor.gearRatio;
        angleGearRatio = config.angleMotor.gearRatio;
        wheelCircumference = config.wheelCircumference;
        driveRotationsToMeters = wheelCircumference/driveGearRatio;

        encoderOffsetDegrees = config.canCoderOffsetDegrees;

        this.fromCenter = config.fromCenter;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond / SwerveConstants.maxDriveVelocity;
        driveDutyCycle.Output = speed;
        driveMotor.setControl(driveDutyCycle);
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        var speed = state.speedMetersPerSecond;
        driveVelocity.Velocity = speed / driveRotationsToMeters;
        driveVelocity.FeedForward = driveFF.calculate(speed);
        driveMotor.setControl(driveVelocity);
    }

    @Override
    public double getDriveVelocity() {
        SmartDashboard.putNumber("driveVelocity", driveMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("driveRotationsToMeters", driveRotationsToMeters);
        return driveMotor.getVelocity().getValueAsDouble() * driveRotationsToMeters;
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * driveRotationsToMeters;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        anglePosition.Position = rotation.getRotations();
        angleMotor.setControl(anglePosition);
    }

    @Override
    public Translation2d getOffset() {
        return fromCenter;
    }

    @Override
    public GenericEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void zeroWheels() {
        //angleMotor.setPosition(Units.degreesToRotations(encoder.getPosition().getDegrees() - encoderOffsetDegrees));
    }

    @Override
    public void configure() {
        //angleMotor.setPosition(Units.degreesToRotations(encoder.getPosition().getDegrees() - encoderOffsetDegrees));
        angleConfig.Feedback.SensorToMechanismRatio = angleGearRatio;
        
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        if(useDriveMotor)
            driveMotor.getConfigurator().apply(driveConfig);
        if(useAngleMotor){
            angleMotor.getConfigurator().apply(angleConfig);
            angleMotor.setPosition(Units.degreesToRotations(-encoder.getPosition().getDegrees() + encoderOffsetDegrees));
        }
    }    
}
