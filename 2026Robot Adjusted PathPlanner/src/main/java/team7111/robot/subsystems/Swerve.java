package team7111.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import team7111.lib.pathfinding.*;
import team7111.robot.Constants.SwerveConstants;
import team7111.robot.utils.SwerveModule;
import team7111.robot.utils.gyro.NavXGyro;
import team7111.robot.utils.gyro.SimSwerveGyro;
import team7111.robot.utils.gyro.GenericGyro;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;

    private final SwerveDrivePoseEstimator swerveOdometry;
    private Field2d field = new Field2d();

    private final GenericGyro gyro;
    private SwerveModuleState[] states = new SwerveModuleState[]{};

    private StructArrayPublisher<SwerveModuleState> commandedStatePublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Commanded Swerve States", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> actualStatePublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Actual Swerve States", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> robotPosePublisher = 
            NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    
    private PathMaster pathMaster = null;

    private SwerveState currentSwerveState = SwerveState.manual;

    private Path path = null;

    private boolean isDriveFieldRelative;

    private DoubleSupplier joystickYTranslation = () -> 0;
    private DoubleSupplier joystickXTranslation = () -> 0;
    private DoubleSupplier joystickYaw = () -> 0;

    private double snapAngleSetpoint = 45;
    private PIDController snapAnglePID;

    private PIDController gamepieceAnglePID;
    private double gamepieceYaw = 0;

    /** X velocity of the robot relative to itself */
    private double robotXVelocity;
    /** Y velocity of the robot relative to itself */
    private double robotYVelocity;
    /** Rotation velocity of the robot, CW positive */
    private double rotationVelocity;

    private SwerveDriveKinematics kinematics;

    private final double controllerDeadzone = 0.0;

    public enum SwerveState{
        initializePath,
        runPath,
        manual,
        stationary,
        x,
        snapAngle,
        bumpAlign,
        followGamePiece
    };

    public Swerve() {
        modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.drivebaseConfig.moduleTypes[0]),
            new SwerveModule(1, SwerveConstants.drivebaseConfig.moduleTypes[1]),
            new SwerveModule(2, SwerveConstants.drivebaseConfig.moduleTypes[2]),
            new SwerveModule(3, SwerveConstants.drivebaseConfig.moduleTypes[3]),
        };

        gyro = RobotBase.isReal()
            ? new NavXGyro()
            : new SimSwerveGyro(this::getStates, SwerveConstants.kinematics);
        gyro.invertYaw(true);
        zeroGyro();

        pathMaster = new PathMaster(this::getPose, () -> getYaw());
        // Senior frog PID
        /*pathMaster.setTranslationPID(3, 0.015215, 0.1012);
        pathMaster.setRotationPID(0.1425, 0.000, 0.00240225);*/

        pathMaster.setTranslationPID(5, 0.01, 0.001);
        pathMaster.setRotationPID(0.25, 0, 0.003);
        pathMaster.setInversions(false, false, RobotBase.isSimulation(), false);

        
        swerveOdometry = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, getYaw(), 
            getPositions(), new Pose2d(4.635 - Units.feetToMeters(20), 4.034536, Rotation2d.fromDegrees(0)));
        
        snapAnglePID = new PIDController(0.25, 0, 0.002);
        gamepieceAnglePID = new PIDController(0.01, 0, 0);

        SmartDashboard.putNumber("SnapAngle P", 0.25);
        SmartDashboard.putNumber("SnapAngle I", 0.0);
        SmartDashboard.putNumber("SnapAngle D", 0.002);

        //Velocity kinematics object
        kinematics = SwerveConstants.kinematics;
        //new SwerveDriveKinematics(modules[0].getOffset(),
        //                                        modules[0].getOffset(),
        //                                        modules[0].getOffset(),
        //                                        modules[0].getOffset());
    }

    @Override 
    public void periodic() {

        SmartDashboard.putNumber("Snap angle", snapAngleSetpoint);

        SmartDashboard.putNumber("Rotation", getYaw().getDegrees());

        snapAnglePID.setP(SmartDashboard.getNumber("SnapAngle P", 0.25));
        snapAnglePID.setI(SmartDashboard.getNumber("SnapAngle I", 0));
        snapAnglePID.setD(SmartDashboard.getNumber("SnapAngle D", 0.002));

        gyro.update();
        swerveOdometry.update(getYaw(), getPositions());
        commandedStatePublisher.set(states);

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getEncoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Meters", mod.getPosition().distanceMeters);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Rotations", mod.getPosition().distanceMeters / SwerveConstants.wheelCircumference);
        }
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        robotPosePublisher.set(getPose());

        actualStatePublisher.set(getStates());

        manageSwerveState();

        SmartDashboard.putString("swerveState", currentSwerveState.name());

        if(DriverStation.getAlliance().isPresent()){
            pathMaster.useAllianceFlipping(DriverStation.getAlliance().get() == Alliance.Red, false);
        }

        SmartDashboard.putNumber("X speed", robotXVelocity);
        SmartDashboard.putNumber("Y speed", robotYVelocity);
    }

    public void simulationPeriodic(){
        for (SwerveModule mod : modules) {
            mod.module.update();
        }
    }

    public void manageSwerveState(){
        switch(currentSwerveState){
            case initializePath:
                if(path == null){
                    setSwerveState(SwerveState.stationary);
                    break;
                }
                pathMaster.initializePath(path);
                setSwerveState(SwerveState.runPath);
                break;
                
            case runPath:
                if(path == null){
                    setSwerveState(SwerveState.initializePath);
                    break;
                }
                if(path.isPathFinished()){
                    setSwerveState(SwerveState.stationary);
                    
                    break;
                }
                pathMaster.periodic(path);
                ChassisSpeeds speeds = pathMaster.getPathSpeeds(path, false, true);
                setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds)); 
                if(path.getWaypoints().length == 0){
                    path = null;
                }
                break;

            case manual:
                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), joystickYaw.getAsDouble(), isDriveFieldRelative, true, false);
                break;
            case stationary:
                manual(0, 0, 0, false, false, false);
                break;
            case x:
                SwerveModuleState[] states = new SwerveModuleState[]{
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135)),
                };
                setModuleStates(states);
                break;
            case snapAngle:
                // Optimize code
                double difference = snapAngleSetpoint - getYaw().getDegrees();
                double simMult = 1.0;
                
                if(difference > 180) {
                    snapAngleSetpoint -= 360;
                } else if(difference < -180) {
                    snapAngleSetpoint += 360;
                }

                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), snapAnglePID.calculate(getYaw().getDegrees() * simMult, snapAngleSetpoint), isDriveFieldRelative, false, false);
                break;
            case followGamePiece:
                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), gamepieceAnglePID.calculate(gamepieceYaw, snapAngleSetpoint), false, false, false);
                break;
            case bumpAlign:
                double angle = joystickYaw.getAsDouble();
                Pose2d pose = getPose();

                if ((pose.getY() > 1.4 && pose.getY() < 3.4) 
                 || (pose.getY() > 4.5 && pose.getY() < 6.5)){
                    if ((pose.getX() > 3.6 && pose.getX() < 5.5) 
                     || (pose.getX() > 11 && pose.getX() < 12.98)){
                        angle = snapAnglePID.calculate(-getYaw().getDegrees(), 45);
                    }
                }

                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), angle, isDriveFieldRelative, false, false);
                break;
            default:
                break;
        }
    }

    public void manual(double forwardBack, double leftRight, double rotation, boolean isFieldRelative, boolean isAngleJoystick, boolean isOpenLoop){
        // Adding deadzone.
        forwardBack = Math.abs(forwardBack) < controllerDeadzone ? 0 : forwardBack;
        leftRight = Math.abs(leftRight) < controllerDeadzone ? 0 : leftRight;
        if(isAngleJoystick)
            rotation = Math.abs(rotation) < controllerDeadzone ? 0 : rotation;

        double hypot = Math.hypot(leftRight, forwardBack);
        if(Math.abs(hypot) > 1){
            hypot = 1;
        }
        hypot = Math.pow(hypot, 3);
        double theta = Math.atan2(forwardBack, leftRight);

        forwardBack = hypot * Math.sin(theta);
        leftRight = hypot * Math.cos(theta);
        // Converting to m/s
        forwardBack *= SwerveConstants.maxDriveVelocity;
        leftRight *= SwerveConstants.maxDriveVelocity;
        if(isAngleJoystick)
            rotation *= SwerveConstants.maxAngularVelocity;

        // Get desired module states.
        ChassisSpeeds chassisSpeeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
            : new ChassisSpeeds(forwardBack, leftRight, rotation);
        
        if(DriverStation.isFMSAttached()){
            if(DriverStation.getAlliance().isPresent()){
                if(DriverStation.getAlliance().get().equals(Alliance.Red)){
                    chassisSpeeds.vxMetersPerSecond *= -1.0;
                    chassisSpeeds.vyMetersPerSecond *= -1.0;
                }
            }
        }

        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(states, isOpenLoop);

        // Converts module states to chassis speeds
        ChassisSpeeds chassisSpeedVelocities = kinematics.toChassisSpeeds(
        modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());

        robotXVelocity = chassisSpeedVelocities.vxMetersPerSecond; 
        robotYVelocity = chassisSpeedVelocities.vyMetersPerSecond;
        rotationVelocity = Units.radiansToDegrees(chassisSpeedVelocities.omegaRadiansPerSecond);
    }

    public void setSwerveState(SwerveState swerveState){
        currentSwerveState = swerveState;
    }

    public SwerveState getSwerveState(){
        return currentSwerveState;
    }
    
    public void setSnapAngle(double snapNumber) {
        snapAngleSetpoint = snapNumber;
    }

    public void setGamepieceYaw(double yaw){
        gamepieceYaw = yaw;
    }

    public void addVisionMeasurement(Pose2d pose, boolean useGyroYaw){
        Rotation2d rotation = useGyroYaw
            ? getYaw()
            : pose.getRotation();
        swerveOdometry.addVisionMeasurement(new Pose2d(pose.getX(), pose.getY(), rotation), Timer.getFPGATimestamp());
    }

    public void displayPathPoses(List<Pose2d> poses){
        field.getObject("Auto Path").setPoses(poses);
    }

    /** To be used by auto. Use the drive method during teleop. */
    public void setModuleStates(SwerveModuleState[] states) {
        setModuleStates(states, false);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        // Makes sure the module states don't exceed the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxDriveVelocity);
        this.states = states;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }
        return currentStates;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getPosition();
        }

        return currentStates;
    }

    public Rotation2d getYaw() {
        return gyro.getYaw(); //Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /** Robot relative, X is forward/back, Y is left/Right, rotation is the robot rotation value */
    public Transform2d getVelocity() {
        if(robotXVelocity > 0.2) {
            robotXVelocity = 0;
        }
        if(robotYVelocity > 0.2) {
            robotYVelocity = 0;
        }

        return new Transform2d(robotXVelocity, robotYVelocity, new Rotation2d(rotationVelocity));
    }

    public void resetOdometry(Pose2d pose) {
        pose = new Pose2d(pose.getX(), pose.getY(), getYaw());
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void setJoysickInputs(DoubleSupplier joystickYTranslation, DoubleSupplier joystickXTranslation, DoubleSupplier joystickYaw){
        this.joystickXTranslation = joystickXTranslation;
        this.joystickYTranslation = joystickYTranslation;
        this.joystickYaw = joystickYaw;
    }

    public void setDriveFieldRelative(boolean isFieldRelative){
        isDriveFieldRelative = isFieldRelative;
    }

    public void zeroGyro() {
        gyro.setYaw(Rotation2d.kZero);
        
    }

    

    public void setPath(Path path){
        this.path = path;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        for (SwerveModule module : modules) {
            builder.addDoubleProperty(
            String.format("Drive Pos %d", module.moduleNumber),
            () -> module.getPosition().distanceMeters,
            null);

            
            builder.addDoubleProperty(
            String.format("Angle %d", module.moduleNumber),
            () -> module.getAngle().getDegrees(),
            null);
        }
    }
}
