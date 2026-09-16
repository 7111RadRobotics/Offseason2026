package team7111.robot.utils.swervemodules;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.Constants.SwerveConstants;
import team7111.robot.utils.config.SwerveModuleConfig;
import team7111.robot.utils.encoder.GenericEncoder;

public class SimSwerveModule implements GenericSwerveModule{
    private DCMotor driveMotorOutput;
    private DCMotor angleMotorOutput;
    private DCMotorSim driveMotorSim;
    private DCMotorSim angleMotorSim;

    private GenericEncoder encoder;

    private double driveMotorAmps;
    private double angleMotorAmps;
    private PIDController drivePID;
    private PIDController anglePID;
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.01, 2.69, 0.19);

    private double driveGearRatio;

    private Translation2d fromCenter;

    public SimSwerveModule(SwerveModuleConfig config){
        driveMotorOutput = config.driveMotor.dcMotor;
        angleMotorOutput = config.angleMotor.dcMotor;

        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorOutput, config.driveMotor.moi, config.driveMotor.gearRatio), 
            driveMotorOutput);
        angleMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(angleMotorOutput, config.angleMotor.moi, config.angleMotor.gearRatio), 
            angleMotorOutput);

        encoder = config.encoder;

        driveGearRatio = config.driveMotor.gearRatio;
        driveMotorAmps = config.driveMotor.currentLimit; //TODO is there some formula/estimation?
        angleMotorAmps = config.angleMotor.currentLimit;
        // constructs a new PID controller for each object. 
        // this avoids the PID from doing too much and trying to compensate for it by causing unwanted behavior
        anglePID = config.angleMotor.pid;
        anglePID = new PIDController(anglePID.getP(), anglePID.getI(), anglePID.getD());
        drivePID = config.driveMotor.pid;
        drivePID = new PIDController(drivePID.getP(), drivePID.getI(), drivePID.getD());

        anglePID.enableContinuousInput(-0.5, 0.5);

        this.fromCenter = config.fromCenter;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        //SmartDashboard.putNumber("setMPS", state.speedMetersPerSecond);
        SmartDashboard.putBoolean("isAtSetpoint", drivePID.atSetpoint());
        double ffCalc = driveFeedforward.calculate(state.speedMetersPerSecond);
        double input = drivePID.calculate(driveMotorSim.getAngularVelocityRPM() / 60, state.speedMetersPerSecond / SwerveConstants.wheelCircumference);
        //input = drivePID.calculate(getDriveVelocity(), 100 * SwerveConstants.wheelCircumference);

        driveMotorSim.setInputVoltage(input);
    }

    @Override
    public double getDriveVelocity() {
        //SmartDashboard.putNumber("driveVel", ((driveMotorSim.getAngularVelocityRadPerSec() / (2*Math.PI)) * SwerveConstants.wheelCircumference));
        SmartDashboard.putNumber("driveVoltage", driveMotorSim.getInputVoltage());
        return ((driveMotorSim.getAngularVelocityRPM() / 60) * SwerveConstants.wheelCircumference);
    }

    @Override
    public double getDrivePosition() {
        return driveMotorSim.getAngularPositionRotations() * SwerveConstants.wheelCircumference;
    }

    @Override
    public Rotation2d getAngle() {
        double rotations = angleMotorSim.getAngularPositionRotations();
        SmartDashboard.putNumber("rotations", rotations);
        return Rotation2d.fromRotations(rotations);
    }

    @Override
    public Translation2d getOffset() {
        return fromCenter;
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        double speed;
        double setpoint = rotation.getRotations();
        anglePID.setSetpoint(setpoint);
        speed = anglePID.calculate(getAngle().getRotations());
        SmartDashboard.putNumber("module angle", getAngle().getDegrees());
        angleMotorSim.setInputVoltage(speed);
    }

    @Override
    public GenericEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void zeroWheels() {
        
    }

    @Override
    public void configure() {

    }

    @Override
    public void update(){
        angleMotorSim.update(0.02);
        driveMotorSim.update(0.02);
    }
    
}
