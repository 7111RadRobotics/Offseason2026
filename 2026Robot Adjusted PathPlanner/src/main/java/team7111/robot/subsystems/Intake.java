package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.ArmSimMotor;
import team7111.robot.utils.motor.CTREMotor;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor.MechanismType;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.TwoMotors;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Intake extends SubsystemBase {
    
    private Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    private MechanismLigament2d intakeLigament = new MechanismLigament2d("Arm", 0.5, 37, 6, new Color8Bit(Color.kBlue));


    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum IntakeState {
        stow,
        deploy,
        intake,
        shoot,
        gyrate,
        manual,
    }

    private double flyWheelSpeed = 0;

    private double pivotPos = 0;
    private double oldPivotPos = 0;

    private double manualIntakeSpeed = 0;

    /** Maximum position in degrees */
    private final double maxPivotPos = 126;

    /** Minimum position in degrees */
    private final double minPivotPos = 0;

    private MotorConfig pivotConfig = new MotorConfig(0.05, 40, false, true, 
                                        new PIDController(0.35, 0.0, 0.001), 
                                        MechanismType.arm, 0.3, 0, 0, 0.0);//2.44, 0.08, 0.52);
    private int pivotID = 12;

    private MotorConfig flyWheelConfig = new MotorConfig(1, 40, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0, 0, 0, 0);
    private int flywheelLeadID = 10;
    private int flywheelFollowID = 11;

    private Motor pivot;

    private Motor flyWheel;

    private Timer timer = new Timer();
    private Timer swapTimer = new Timer();
    
    private boolean swapIntake = false;

    private boolean intakeHasReachedSetpoint = false;

    private IntakeState currentState = IntakeState.stow;

    public Intake() {
        //TODO set CTRE motor ID to a real ID
        pivot = RobotBase.isReal()
            ? new REVMotor(pivotID, null, pivotConfig)
            : new ArmSimMotor(
                null,
                new SingleJointedArmSim(
                    DCMotor.getNEO(1), pivotConfig.gearRatio, 0.10849, 0.2, 
                    Degrees.of(minPivotPos).in(Radians), Degrees.of(maxPivotPos).in(Radians), true, Degrees.of(minPivotPos).in(Radians)), 
                pivotConfig.pid, 
                pivotConfig.armFF);

        //TODO set REV motor ID to a real ID
        flyWheel = RobotBase.isReal()
            ? new TwoMotors(
                new REVMotor(flywheelLeadID, null, flyWheelConfig), 
                new REVMotor(flywheelFollowID, null, flyWheelConfig), flywheelLeadID, false)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.01, flyWheelConfig.gearRatio), DCMotor.getNEO(2), 0.1),
                flyWheelConfig.pid,
                flyWheelConfig.simpleFF);
        
        mechanism2d.getRoot("Intake", 0.7, 0.3).append(intakeLigament);
        Shuffleboard.getTab("Mechanisms").add("intake", mechanism2d);

        pivot.setPositionReadout(0);
        pivot.setSpeedLimits(3, -3, true);
    }

    public void periodic(){
        manageState();

        if(pivotPos >= maxPivotPos) {
            pivotPos = maxPivotPos;
        }

        if(pivotPos <= minPivotPos) {
            pivotPos = minPivotPos;
        }
        
        flyWheel.setDutyCycle(flyWheelSpeed);

        /*if(oldPivotPos != pivotPos) {
                intakeHasReachedSetpoint = false;
        }*/
        if ((pivot.getPosition() >= pivotPos -2.5 && pivot.getPosition() <= pivotPos + 2.5)
          || (pivot.getPosition() > maxPivotPos - 2.0 && (currentState.equals(IntakeState.deploy) || currentState.equals(IntakeState.intake)))) {
            if(!currentState.equals(IntakeState.intake)){
                pivot.setVoltage(0);
            }else{
                pivot.setVoltage(1.0);
            }
                
            intakeHasReachedSetpoint = true;
            
            
        } else {
            pivot.setSetpoint(pivotPos, true);
        }
        //oldPivotPos = pivotPos;*/

        
        flyWheel.periodic();
        pivot.periodic();
        intakeLigament.setAngle(-pivot.getPosition() + maxPivotPos);

        if (currentState != IntakeState.gyrate) {
            timer.stop();
            timer.reset();
            swapTimer.stop();
            swapTimer.reset();
          
        }

        SmartDashboard.putNumber("intake pivot position", pivot.getPosition());
        SmartDashboard.putNumber("Intake Setpoint", pivotPos);
        SmartDashboard.putBoolean("Intake is at Setpoint", pivot.isAtSetpoint(20));
        SmartDashboard.putNumber("Intake Velocity", pivot.getVelocity());
        SmartDashboard.putNumber("Intake Voltage", pivot.getVoltage());
        SmartDashboard.putNumber("Intake Duty Cycle", pivot.getDutyCycle());
        SmartDashboard.putString("Intake State", currentState.toString());
        SmartDashboard.putNumber("Intake State Timer", timer.get());
   
        
    }
 
    public void simulationPeriodic(){}

    // The below methods are examples of retrieveable boolean values.
    // These can be checked in SuperStructure to determine a SuperState
    // or change a state/value in another subsystem.
    public boolean isAtSetpoint(){
        boolean isAtSetpoint = pivot.isAtSetpoint(2); // would be true if mechanisms were at/near their setpoints.
        
        return isAtSetpoint;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case stow:
                stow();
                break;
            case deploy:
                deploy();
                break;
            case intake:
                intake();
                break;
            case shoot:
                shoot();
                break;
            case gyrate:
                gyrate();
                break;
            case manual:
                manual();
                break;
            default:
                break;
        }
    }

    private void stow(){
        //System.out.println("Runs code for the stow state");
        pivotPos = minPivotPos;
        flyWheelSpeed = 0;
        if(pivot.getPosition() <= pivotPos + 2.5){
            pivot.setVoltage(0);
        }
    }

    private void deploy(){
        pivotPos = maxPivotPos;
        flyWheelSpeed = 0;

    }

    private void intake(){
        pivotPos = maxPivotPos;
        flyWheelSpeed = -0.7;

        if(pivot.isAtSetpoint(2)){
            //pivot.setVoltage(-1);
        }
    }

    private void shoot(){
        pivotPos = 50;
        flyWheelSpeed = 0;

    }

    private void gyrate(){
        
        


        if (pivot.isAtSetpoint(2.5) || timeDelay(timer, 1.5)) {
            swapIntake = true;
            
        } 

        if (swapIntake) {
            if (timeDelay(swapTimer, 1)) {
            
                if (pivotPos == maxPivotPos) {
                    pivotPos = 50;
                    swapIntake = false;
                    flyWheelSpeed = 0;
                } else if (pivotPos == 50) {
                    pivotPos = maxPivotPos;
                    swapIntake = false;
                    flyWheelSpeed = -0.7;
                } else {
                    pivotPos = maxPivotPos;
                    swapIntake = false;
                    flyWheelSpeed = 0;
                }
            }

        }
    }

    private void manual(){
        //System.out.println("Runs code for the manual state");

        flyWheelSpeed = manualIntakeSpeed;
    }

    public void updateManualSpeed(double manualSpeed) {
        manualIntakeSpeed = manualSpeed;
    }

    public void setState(IntakeState state){
        this.currentState = state;
    }

    public IntakeState getState(){
        return currentState;
    }

    public double getPosition(){
        return pivot.getPosition();
    }
    public double getSetpoint(){
        return pivotPos;
    }

    public void setPosition(double pos){
        pivotPos = pos;

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
