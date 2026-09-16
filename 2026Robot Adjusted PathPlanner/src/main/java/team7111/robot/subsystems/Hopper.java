package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.Motor.MechanismType;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.TwoMotors;
import team7111.robot.Constants.MechanismConstants;
import team7111.robot.utils.motor.CTREMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Hopper extends SubsystemBase {
    
    private double manSpindexerSpeed = 0.0;
    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum HopperState {
        intake,
        shoot,
        unjam,
        stopped,
        manual,
    }

    private HopperState currentState = HopperState.stopped;

    private MotorConfig spindexerConfig = new MotorConfig(
        1, 40, true, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0.001, 0, 0, 0
    );
    private MotorConfig shooterIndexerConfig = new MotorConfig(
        1, 20, false, true, new PIDController(0.007, 0, 0), MechanismType.flywheel, 0.208, 0.115, 0, 0
    );

    private Motor spindexer;
    private Motor shooterIndexer;

    private double spindexerSpeed = 0;
    private double shooterIndexerSpeed = 0;

    private double indexerSetpoint = 0;

    private Timer timer = new Timer();

    private boolean isFollowingShooter = false;
    private boolean isFollowerInverted = false;

    public Hopper() {
        spindexer = RobotBase.isReal()
            ? new CTREMotor(14, null, spindexerConfig)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.01, spindexerConfig.gearRatio), DCMotor.getNEO(1), 0.1),
                spindexerConfig.pid,
                spindexerConfig.simpleFF
            );
        shooterIndexer = RobotBase.isReal()
            ? new CTREMotor(15, null, shooterIndexerConfig)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 0.01, shooterIndexerConfig.gearRatio), DCMotor.getKrakenX60(1), 0.1),
                shooterIndexerConfig.pid,
                shooterIndexerConfig.simpleFF
            );

            shooterIndexer.setSpeedLimits(6500, -6500, false);
    }

    public void periodic(){
        manageState();

        spindexer.setDutyCycle(spindexerSpeed);
        /*if(shooterIndexer.getVelocity() > 30 && shooterIndexerSpeed < 0){
            shooterIndexer.setVoltage(0);
        }else if(shooterIndexer.getVelocity() < -30 && shooterIndexerSpeed > 0){
            shooterIndexer.setVoltage(0);
        }else*/
        
        if(!isFollowingShooter){
            shooterIndexer.setFollower(false, MechanismConstants.shooterID, isFollowerInverted);
            shooterIndexer.setDutyCycle(shooterIndexerSpeed);
        }else{
            shooterIndexer.setVelocity(indexerSetpoint);
            //shooterIndexer.setFollower(true, MechanismConstants.shooterID, isFollowerInverted);
        }
        shooterIndexer.periodic();

        SmartDashboard.putNumber("Spindexer RPM", spindexer.getVelocity());
        SmartDashboard.putString("Hopper State", currentState.toString());
        SmartDashboard.putNumber("Spindexer Current", spindexer.getCurrent());
    }

    public void simulationPeriodic(){

    }

    public boolean getBeamBreak(){
        //TODO add beam break object to hopper and return it's value here
        return false;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case unjam:
                unJam();
                break;
            case intake:
                intake();
                break;
            case shoot:
                shoot();
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

    // named differently to not overide a different method
    private void unJam(){
        shooterIndexerSpeed = 0.0;
        //spindexerSpeed = -0.2;//0.36;
        isFollowingShooter = true;
        if (!timeDelay(timer, 1)) {
                spindexerSpeed = -0.2;
        } else {
            spindexerSpeed = 0;
        }
    }

    private void intake(){
        shooterIndexerSpeed = 0.0;
        spindexerSpeed = 0.0;//0.36;
        isFollowingShooter = false;
    }

    private void shoot(){

        if (spindexer.getCurrent() >= 100) {
            currentState = HopperState.unjam;
            unJam();
            return;
        }
        shooterIndexerSpeed = 0.4;
        spindexerSpeed = 0.51;

        

        isFollowingShooter = true;
    }

    private void stopped(){
        shooterIndexerSpeed = 0;
        spindexerSpeed = 0;
        isFollowingShooter = false;
    }

    private void manual(){
        spindexerSpeed = manSpindexerSpeed;
        isFollowingShooter = false;
    }

    /** Updates the manual speed setpoint in rpm */
    public void updateManualSpeed(double speed) {
        manSpindexerSpeed = speed;
    }

    public void setState(HopperState state){
        this.currentState = state;
    }

    public HopperState getState(){
        return currentState;
    }

    public void setSpeed(double speed){
        spindexerSpeed = speed;
        shooterIndexerSpeed = speed;
    }

    public double getSpeed(){
        return spindexer.getDutyCycle();
    }

    public void giveShooterSpeed(double rpm){
        indexerSetpoint = rpm * 1.9;
    }

    private boolean timeDelay(Timer timer, double delay){
        timer.start();
        if (timer.hasElapsed(delay)) {
            timer.reset();
            timer.stop();
            return true;
        }
        return false;
    }
}
