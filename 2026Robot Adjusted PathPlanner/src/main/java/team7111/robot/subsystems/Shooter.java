package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.Constants.MechanismConstants;
import team7111.robot.subsystems.Aimbot.shotType;
import team7111.robot.utils.encoder.RelativeThroughBore;
import team7111.robot.utils.motor.ArmSimMotor;
import team7111.robot.utils.motor.CTREMotor;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.TwoMotors;
import team7111.robot.utils.motor.Motor.MechanismType;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Shooter extends SubsystemBase {
    
    public enum ShooterState {
        idle,
        score,
        scoreAimbot,
        followAimbot,
        pass,
        stopped,
        manual,
    }

    private Aimbot aimbot;

    private Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    private MechanismLigament2d hoodTrajectoryLigament = 
        new MechanismLigament2d("Trajectory", 1.5, 37, 2, new Color8Bit(Color.kOrange));
    private MechanismLigament2d hoodPositionLigament = 
        new MechanismLigament2d("Position", 0.25, 0, 5, new Color8Bit(Color.kCyan));

    private MotorConfig hoodConfig = new MotorConfig(
        105.6, 20, false, false, new PIDController(0.725, 0.003, 0.00035),
        MechanismType.arm, 0.0, 0.0, 0, 0);

    private MotorConfig flywheelConfig = new MotorConfig(
        1, 40, false, false, new PIDController(0.0001, 0.0, 0.0), // 0.51
        MechanismType.flywheel, 0.19, 0.115, 0, 0);//0.21, 0.19, 1.66, 0); //0.184

    private Motor hood;
    private Motor flywheels;

    private double hoodTrajSetpoint = 82;
    private double flywheelSpeed = 0;

    private final double maxHoodPos = MechanismConstants.maxHoodPos;//30.962;
    private final double minHoodPos = MechanismConstants.minHoodPos;//7
    private final double maxHoodTraj = MechanismConstants.maxHoodTraj; // 82 //83
    private final double minHoodTraj = MechanismConstants.minHoodTraj; // 60 //59.038

    private ShooterState currentState = ShooterState.stopped;

    public Shooter(Aimbot aimbot) {
        this.aimbot = aimbot;

        double hoodMOI = 0.0708190376784300761;

        hood = RobotBase.isReal()
            ? new CTREMotor(18, new RelativeThroughBore(0, 1, true, 17.5, minHoodPos - 1.0), hoodConfig)
            : new ArmSimMotor(
                null,
                new SingleJointedArmSim(
                    DCMotor.getKrakenX60(1), hoodConfig.gearRatio, hoodMOI, 0.2, 
                    Degrees.of(minHoodPos).in(Radians), Degrees.of(maxHoodPos).in(Radians), true, Degrees.of(minHoodPos).in(Radians)), 
                hoodConfig.pid, 
                hoodConfig.armFF);
        
        flywheels = RobotBase.isReal()
            ? new TwoMotors(
                new CTREMotor(MechanismConstants.shooterID, null, flywheelConfig), 
                new CTREMotor(MechanismConstants.shooterFollowerID, null, flywheelConfig),
                MechanismConstants.shooterID, true)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.01, 1), DCMotor.getKrakenX60(2), 0.1),
                flywheelConfig.pid,
                flywheelConfig.simpleFF);

        mechanism2d.getRoot("Shooter hood", 0.25, 0.35).append(hoodTrajectoryLigament);
        mechanism2d.getRoot("Shooter hood", 0.25, 0.5).append(hoodPositionLigament);

        Shuffleboard.getTab("Mechanisms").add("Shooter", mechanism2d);

        flywheels.setSpeedLimits(6000, -6000, false);
    }

    public void periodic(){
        manageState();
        hood.periodic();
        flywheels.periodic();

        if(hoodTrajSetpoint > maxHoodTraj){
            hood.setSetpoint(minHoodPos, false);
        }else if(hoodTrajSetpoint < minHoodTraj){
            hood.setSetpoint(maxHoodPos, false);
        }else{
            hood.setSetpoint(90 - hoodTrajSetpoint, false);
        }
        if(flywheelSpeed == 0 || flywheels.getVelocity() >= 75 + flywheelSpeed){
            flywheels.setVoltage(0);
        }else 
            flywheels.setVelocity(flywheelSpeed);

           // flywheels.setVoltage(0.18325);
            
        

        hoodTrajectoryLigament.setAngle(90 - hood.getPosition());
        hoodPositionLigament.setAngle(-hood.getPosition() + 180);
        SmartDashboard.putNumber("hood position", hood.getPosition());
        SmartDashboard.putNumber("hood setpoint", hoodTrajSetpoint);
        SmartDashboard.putNumber("hood trajectory", -hood.getPosition() + 90);
        SmartDashboard.putNumber("Flywheel Velocity", flywheels.getVelocity());
        SmartDashboard.putNumber("FlywheelSetpoint", flywheelSpeed);
        SmartDashboard.putNumber("Flywheel Voltage", flywheels.getVoltage());
        SmartDashboard.putBoolean("is Shooter At Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("is Shooter At Velocity Setpoint", isAtSpeedSetpoint());
    }

    public void simulationPeriodic(){}

    // The below methods are examples of retrieveable boolean values.
    // These can be checked in SuperStructure to determine a SuperState
    // or change a state/value in another subsystem.
    public boolean isAtSetpoint(){
        boolean isAtSetpoint = hood.isAtSetpoint(2.5); 
        return isAtSetpoint;
    }

    public boolean isAtSpeedSetpoint(){
        boolean isAtSetpoint = flywheels.isAtVelocitySetpoint(40);
        return isAtSetpoint;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case idle:
                idleMode();
                break;
            case pass:
                pass();
                break;
            case score:
                score();
                break;
            case scoreAimbot:
                scoreAimbot();
                break;
            case followAimbot:
                followAimbot();
                break;
            case stopped:
                stopped();
                break;
            case manual:
                manual();
                break;
            default:
                break;
        }
    }

    private void idleMode(){
        hoodTrajSetpoint = 23;
        flywheelSpeed = 1000;
    }

    private void pass(){}

    private void score(){}

    private void scoreAimbot(){
        aimbot.setShotType(shotType.Parabolic);
        hoodTrajSetpoint = aimbot.getCalculatedAngle();
        flywheelSpeed = aimbot.getCalculatedSpeed();
    }

    private void followAimbot(){
        hoodTrajSetpoint = aimbot.getCalculatedAngle();
        flywheelSpeed = aimbot.getCalculatedSpeed();
    }

    private void stopped(){
        hoodTrajSetpoint = 81;
        flywheelSpeed = 0;
    }

    private void manual(){
        aimbot.setShotType(shotType.Manual);
        hoodTrajSetpoint = aimbot.getCalculatedAngle();
        flywheelSpeed = aimbot.getCalculatedSpeed();
    }

    public void setState(ShooterState state){
        this.currentState = state;
    }

    public ShooterState getState(){
        return currentState;
    }
}
