package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.element.ModuleElement.DirectiveKind;
import javax.print.attribute.standard.RequestingUserName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.subsystems.Aimbot.shotType;
import team7111.robot.subsystems.Aimbot.presetShotType;
import team7111.robot.subsystems.Autonomous.Autos;
import team7111.robot.subsystems.Autonomous.Paths;
import team7111.robot.subsystems.Hopper.HopperState;
import team7111.robot.subsystems.Intake.IntakeState;
import team7111.robot.subsystems.Shooter.ShooterState;
import team7111.robot.subsystems.Swerve.SwerveState;
import team7111.robot.utils.AutoAction;
import team7111.lib.pathfinding.*;

/**
 * This class handles the overall state of the robot.
 * It does this by defining various "SuperState" values and calling the methods assosiated with each one.
 * Each method uses logic to determine subsystem states, when to switch SuperStates, and what SuperState to go to next.
 * Logic used can be button inputs, subsystem states, or other subsystem conditions.
 */
public class SuperStructure extends SubsystemBase {
    /**
     * This enumeration contains the values or "states" which determine the subsystems.
     * The top 2 state names are temporary states for testing 
     */
    public enum SuperState {
        stowed,
        deployed,
        intake,
        pass,
        score,
        snowBlowerPass,
        snowBlowerScore,
        prepareHubShot,
        preparePass,
        autonomousEnter,
        autonomous,
        autonomousExit,
        manual,
    }

    //Subsystem Variables
    private final Autonomous auto;
    private final Swerve swerve;
    private final Vision vision;
    public final Aimbot targeting;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;
    private final Field field;

    // Buttons of controllers can be assigned to booleans which are checked in various super states. 
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /** This represents the current superstate of the robot */
    private SuperState superState = SuperState.stowed;
    private boolean hasAcheivedState = false;

    private boolean inAuto = false;
    private int autoIndex = 0;
    private List<AutoAction> autoActions;
    private double matchTime;

    private boolean alignToHub = false;
    private boolean moveThroughTrench = false;
    private boolean orientWithBump = false;
    private boolean useObjectDetection = false;
    private boolean defaultDrive = true;
    private boolean useXWheels = false;
    
    private boolean intaking = false;
    private boolean scoring = false;
    private boolean passing = false;
    private boolean stow = false;

    private boolean autoTargeting = true;

    private shotType currentShot = shotType.Transport;
    private shotType scoringState = shotType.Parabolic;

    private String gameData;
    private boolean bHub = false;
    private boolean hasData = false;
    private Autos currentAutos;

    private boolean operatorDisabled = false;

    private Timer phaseTimer = new Timer();

    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(Autonomous auto, Swerve swerve, Vision vision, Aimbot aimbot, Intake intake, Hopper hopper, Shooter shooter, Field field){
        this.auto = auto;
        this.swerve = swerve;
        this.vision = vision;
        this.targeting = aimbot;
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        this.field = field;
        currentAutos = auto.getSelectedAuto();
        autoActions = auto.getAutonomous(auto.getSelectedAuto());

        DriverStation.silenceJoystickConnectionWarning(true);
        this.swerve.setJoysickInputs(() -> -driverController.getLeftX(), () -> -driverController.getLeftY(), () -> -driverController.getRightX());
        this.swerve.setDriveFieldRelative(true);
        this.swerve.setSwerveState(SwerveState.manual);

        targeting.giveResources(operatorController, () -> {
            if(DriverStation.getAlliance().isPresent())
                return DriverStation.getAlliance().get() == Alliance.Blue;
            return true;
        });

        auto.giveResources(this);

    }

    /**
     * This method runs every iteration (every 20ms). Actions like state management and stateless logic are run here.
     */
    public void periodic(){
        /**
         * intake right trigger
         * score left trigger
         * pass left bumper
         */
        gameData = DriverStation.getGameSpecificMessage();
        if(!hasData && !gameData.isEmpty()) {
            if(gameData.charAt(0) == 'B') {
                    bHub = false;
                    hasData = true;
                } else if(gameData.charAt(0) == 'R') {
                    bHub = true;
                    hasData = true;
                } else {
                    hasData = false;
                }
        }

        double phaseLength = 25.0;
        
        if (DriverStation.isTeleopEnabled() && hasData) {
            if(DriverStation.getMatchTime() <= 130)
                phaseTimer.start();
        } else if (DriverStation.isDisabled()) {
            phaseTimer.stop();
            phaseTimer.reset();
        }
        
        if(phaseTimer.get() >= 20 && phaseTimer.get() <= 21) {
           
            driverController.setRumble(RumbleType.kBothRumble, 1);            
        } else {
            driverController.setRumble(RumbleType.kBothRumble, 0);
        }
        
        if(DriverStation.getMatchTime() >= 29.9 || DriverStation.getMatchTime() < 0){
            if (phaseTimer.hasElapsed(25)) {
                bHub = !bHub;
                
                phaseTimer.restart();
            }
        }else{
            if(DriverStation.getAlliance().isPresent()){
                bHub = DriverStation.getAlliance().get().equals(Alliance.Blue);
            }
            phaseLength = 30.0;
        }

        hopper.giveShooterSpeed(targeting.getCalculatedSpeed());
        
        //Timer for the periodic
        long startTime = System.nanoTime();
        SmartDashboard.putString("SuperState", superState.name());

        SmartDashboard.putBoolean("Active Hub", bHub);
        SmartDashboard.putString("gameData", gameData);
        SmartDashboard.putNumber("Phase Time", phaseLength - phaseTimer.get());

        SmartDashboard.putBoolean("roboPoseIsNull", vision.getRobotPose(0.1) == null);
        final boolean useOneCamera = false; // determine whether true or false
        boolean hasAppliedVisionMeasurement = false;
        Pose3d visionRobotPoseShooter = vision.getRobotPose(vision.shooterCam, 0.15);
        Pose3d visionRobotPoseClimb = vision.getRobotPose(vision.climberCam, 0.1);
        Pose3d bestVisionRobotPose = vision.getBestRobotPose(0.2);
        matchTime = DriverStation.getMatchTime();

        // if(visionRobotPoseClimb != null && RobotBase.isReal()){
        //     swerve.addVisionMeasurement(visionRobotPoseClimb.toPose2d(), true);
        //     hasAppliedVisionMeasurement = true;
        // } else if(visionRobotPoseShooter != null && RobotBase.isReal()){
        //     if(!useOneCamera && !hasAppliedVisionMeasurement)
        //         swerve.addVisionMeasurement(visionRobotPoseShooter.toPose2d(), true);
        boolean useAssumedPose = true;
        if(bestVisionRobotPose != null){
            useAssumedPose = false;
            swerve.addVisionMeasurement(bestVisionRobotPose.toPose2d(), true);
        }

        if((matchTime <= 0 && DriverStation.isDisabled()) || driverController.getYButtonPressed()){
            Autos selectedAuto = auto.getSelectedAuto();
            if (!currentAutos.equals(selectedAuto) || driverController.getYButtonPressed()){
                autoActions = auto.getAutonomous(selectedAuto);
                if(useAssumedPose)
                    swerve.resetOdometry(auto.getAssumedPose());
                ArrayList<Pose2d> pathPoses = new ArrayList<>();
                pathPoses.add(auto.getAssumedPose());
                for (AutoAction action : autoActions) {
                    if(action.isPath()){
                        Path displayPath = action.getAsPath();
                        if(DriverStation.getAlliance().isPresent()){
                            if(DriverStation.getAlliance().get().equals(Alliance.Red)){
                                Path flippedPath = new Path(action.getAsPath().getWaypoints());
                                //flippedPath.flipPath(true, false);
                                displayPath = flippedPath;
                            }
                        }
                        for (Waypoint waypoint : displayPath.getWaypoints()) {
                            pathPoses.add(waypoint.getPose());
                        }
                        //displayPath.flipPath(false, false);
                    }
                }
                swerve.displayPathPoses(pathPoses);
                currentAutos = selectedAuto;
            }
        }
        SmartDashboard.putString("SuperStructure Auto", currentAutos.name());

        // Driver controller commands
        /* Current plan for driver controls:
            Left Trigger: Intake
            Left Bumper: Intake with Object Detection
            Left Back: Retract Intake
            Right Trigger: Shoot in Hub (with aiming)
            Right Bumper: Pass/HubPreset
            Right Back: Align to hub preset (nearest of 5)
            A button: Bump Align
         * 
         */
        if(driverController.getStartButton()) {
            swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            //swerve.zeroGyro();
            //swerve.resetOdometry(new Pose2d(0, 0, swerve.getYaw()));
        }

        if(driverController.getBackButtonPressed()) {
            swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        }

        // move through trench button commented due to drivers request
        /*if(driverController.getLeftTriggerAxis() > 0.15 && !moveThroughTrench){
            moveThroughTrench = true;
            swerve.setPath(auto.getNearestTrenchPath(swerve.getPose()));
            swerve.setSwerveState(SwerveState.initializePath);
        }else if(driverController.getLeftTriggerAxis() <= 0.15 && moveThroughTrench){
            moveThroughTrench = false;
        }*/

        // Driver controller commands
        stow = driverController.getBButton();

        if(driverController.getLeftBumperButtonPressed()) {
            //intaking = true;
            //useObjectDetection = true;
            useXWheels = true;
        } else if(driverController.getLeftBumperButtonReleased()) {
            //intaking = false;
            //useObjectDetection = false;
            useXWheels = false;
            if(superState == SuperState.score || superState == SuperState.snowBlowerScore || superState == SuperState.prepareHubShot){
                swerve.setSwerveState(SwerveState.snapAngle);
            }
        }

        if(driverController.getLeftTriggerAxis() > 0.1 && !intaking) {
            intaking = true;
        }else if(driverController.getLeftTriggerAxis() <= 0.1 && intaking && !useObjectDetection){
            intaking = false;
        }

        if(driverController.getLeftStickButton()){
            intaking = false;
        }

        if(driverController.getRightBumperButtonPressed()) {
            passing = true;
            targeting.setPreset(presetShotType.RegHubShot);
        } else if(driverController.getRightBumperButtonReleased()) {
            passing = false;
        }

        if(driverController.getRightTriggerAxis() > 0.1 && !scoring) {
            scoring = true;
            
        }else if(driverController.getRightTriggerAxis() <= 0.1 && scoring){
            scoring = false;
        }

        if(driverController.getRightStickButtonPressed()) {
            targeting.setPreset(presetShotType.Trench);
            passing = true;
        } else if(driverController.getRightStickButtonReleased()) {
            passing = false;
        }

        if(driverController.getLeftStickButtonPressed()) {
            targeting.setPreset(presetShotType.Tower);
            passing = true;
        } else if(driverController.getLeftStickButtonReleased()) {
            passing = false;
        }

        if(driverController.getAButtonPressed()) {
            orientWithBump = true;
            swerve.setSnapAngle(45);
            swerve.setSwerveState(SwerveState.snapAngle);
        } else if(driverController.getAButtonReleased()) {
            orientWithBump = false;
        }
        if(driverController.getXButtonPressed()) {
            alignToHub = true;
            swerve.setPath(auto.getNearestHubScoringPath(swerve.getPose()));
            swerve.setSwerveState(SwerveState.initializePath);
        } else if(driverController.getXButtonReleased()) {
            alignToHub = false;
        }

        if(useObjectDetection){
            swerve.setGamepieceYaw(vision.getGamepieceYaw());
        }

        defaultDrive = !scoring && !alignToHub && !moveThroughTrench && !orientWithBump && !useObjectDetection;

        if(defaultDrive && !inAuto){
            swerve.setSwerveState(SwerveState.manual);
        }
        

        // Operator controller commands
        /*if(operatorController.getStartButtonPressed()) {
            if(!operatorDisabled) {
                operatorDisabled = true;
                targeting.toggle();
                setSuperState(SuperState.manual);
                hopper.setState(HopperState.stopped);
                intake.setState(IntakeState.stow);
                shooter.setState(ShooterState.followAimbot);
            } else {
                setSuperState(SuperState.deployed);
                operatorDisabled = false;
            }
            
        }*/
        if(operatorController.getAButtonPressed()) {
            scoringState = shotType.ShotTable;
        }
        if(operatorController.getBButtonPressed()) {
            scoringState = shotType.ShootOnTheMove;
        }
        if(operatorController.getXButtonPressed()) {
            scoringState = shotType.Parabolic;
        }
        
        if(operatorController.getBackButtonPressed()) {
            autoTargeting = !autoTargeting;
        }
        
        hasAcheivedState = manageSuperState(superState);

        //Overrides any states for hopper and intake manual

        if(useXWheels){
            swerve.setSwerveState(SwerveState.x);
        }

        if(operatorController.getPOV() != -1) {
            hopper.setState(HopperState.manual);
            
            switch (operatorController.getPOV()) {
                case 90:
                    hopper.updateManualSpeed(-0.1);
                    break;
                case 270:
                    hopper.updateManualSpeed(0.6);
                    break;
                default:
                    break;
            }
        }

        if(operatorController.getRightTriggerAxis() > 0.15) {
            intake.setState(IntakeState.manual);

            intake.updateManualSpeed(operatorController.getRightTriggerAxis());
        }
        if(operatorController.getLeftTriggerAxis() > 0.15) {
            intake.setState(IntakeState.manual);

            intake.updateManualSpeed(-operatorController.getLeftTriggerAxis());
        }
        //If autotargeting, will check if the robot is in the nuteral zone and set to shoot towards the corners
        //if(autoTargeting) {
            if((field.inAllianceZone(swerve.getPose()) && autoTargeting) || (!autoTargeting && !operatorController.getLeftBumperButton() && !operatorController.getRightBumperButton())) {
                targeting.resetTarget();
            } else {
                Pose3d corner = null;
                if(DriverStation.isDSAttached()) {
                    if(DriverStation.getAlliance().isPresent()) {
                        if(DriverStation.getAlliance().get() == Alliance.Blue) {
                            corner = new Pose3d(1.0, 4.034536, 0, null);
                        } else {
                            corner = new Pose3d(16.540988-1.0, 4.034536, 0, null);
                        }
                    }
                    if((swerve.getPose().getY() >= Units.inchesToMeters(317.69/2) && autoTargeting) || (!autoTargeting && operatorController.getLeftBumperButton())) {
                        targeting.setCustomTarget(new Pose3d(corner.getX(), corner.getY() + Units.inchesToMeters(307.69) / 4, corner.getZ(), null));
                    } else {
                        targeting.setCustomTarget(new Pose3d(corner.getX(), corner.getY() - Units.inchesToMeters(307.69) / 4, corner.getZ(), null));
                    }
                }
            }
        //}

        SmartDashboard.putNumber("ShooterAngle", targeting.getCalculatedAngle());
        SmartDashboard.putNumber("ShooterSpeed", targeting.getCalculatedSpeed());

        SmartDashboard.putString("ShootingType", scoringState.toString());
        long endTime = System.nanoTime();

        //Timing measurement, in milliseconds
        SmartDashboard.putNumber("Time for superstructure periodic", (double) ((endTime - startTime) / 1000000.0));
        SmartDashboard.putBoolean("manual mode", superState == SuperState.manual);
    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){}

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * <p>Each state will have its own case statement, returning its state method.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case stowed:
                return stowed();
            case deployed:
                return deployed();
            case score:
                return score();
            case intake:
                return intake();
            case preparePass:
                return preparePass();
            case pass:
                return pass();
            case snowBlowerPass:
                return snowBlowerPass();
            case snowBlowerScore:
                return snowBlowerScore();
            case prepareHubShot:
                return prepareHubShot();
            case manual:
                return manual();
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            
            default:
                return defaultState(state);
        }
    }

    /**
     * Each of these methods, called "state methods", represent a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */
    private boolean stowed(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Transport);
        shooter.setState(ShooterState.followAimbot);
        intake.setState(IntakeState.stow);
        hopper.setState(HopperState.stopped);

        if(intaking) {
            setSuperState(SuperState.deployed);
        }
        return intake.isAtSetpoint() && shooter.isAtSetpoint();
    }

    private boolean deployed(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Transport);
        shooter.setState(ShooterState.followAimbot);
        if(intaking){
            setSuperState(SuperState.intake);
            return true;
        }

        if(hopper.getBeamBreak() || stow){
            setSuperState(SuperState.stowed);
            return true;
        }
        targeting.setShotType(shotType.Transport);
        
        intake.setState(IntakeState.deploy);
        hopper.setState(HopperState.stopped);

        if(passing) {
            setSuperState(SuperState.preparePass);
        }
        else if(scoring) {
            setSuperState(SuperState.prepareHubShot);
        }

        return intake.isAtSetpoint() && shooter.isAtSetpoint();
    }

    private boolean intake(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Transport);
        shooter.setState(ShooterState.followAimbot);
        intake.setState(IntakeState.intake);
        hopper.setState(HopperState.intake);
        
        if(useObjectDetection){
            swerve.setSwerveState(SwerveState.followGamePiece);
        }

        if(!intaking) {
            setSuperState(SuperState.deployed);
        }

        if(passing) {
            setSuperState(SuperState.preparePass);
        } else if(scoring) {
            setSuperState(SuperState.prepareHubShot);
        }
        
        return intake.isAtSetpoint();
    }

    private boolean preparePass() {
        targeting.setToggle(true);
        hopper.setState(HopperState.stopped);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            if(!targeting.getShotType().equals(shotType.Preset)){
                targeting.setShotType(shotType.Preset);
                return true;
            }
            targeting.setShotType(shotType.Preset);
        }
        
        
        if(shooter.isAtSetpoint() && shooter.isAtSpeedSetpoint()) {
            if(intaking) {
                setSuperState(SuperState.snowBlowerPass);
                return true;
            }
            setSuperState(SuperState.pass);
            return true;
        }

        if(!passing) {
            setSuperState(SuperState.deployed);
        }
        return false;
    }

    private boolean pass(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Preset);
        shooter.setState(ShooterState.followAimbot);
        intake.setState(IntakeState.gyrate);
        hopper.setState(HopperState.shoot);
        if(intaking){
            setSuperState(SuperState.snowBlowerPass);
        }
        if(!passing){
            setSuperState(SuperState.deployed);
        }
        return shooter.isAtSetpoint();
    }

    private boolean snowBlowerPass(){
        targeting.setToggle(true);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(shotType.Preset);
        }
        intake.setState(IntakeState.intake);
        hopper.setState(HopperState.shoot);
        if(!intaking){
            setSuperState(SuperState.pass);
        }

        if(!passing) {
            setSuperState(SuperState.deployed);
        }
        return shooter.isAtSetpoint();
    }

    private boolean prepareHubShot(){
        targeting.setToggle(true);
        ShooterState shooterState = ShooterState.followAimbot;
        swerve.setSnapAngle(targeting.getCalculatedDirection());
        swerve.setSwerveState(SwerveState.snapAngle);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            if(!targeting.getShotType().equals(scoringState)){
                targeting.setShotType(scoringState);
                return true;
            }
            targeting.setShotType(scoringState);
        }
        shooter.setState(shooterState);

        if(shooter.isAtSetpoint() && shooter.isAtSpeedSetpoint()) {
            if(intaking) {
                setSuperState(SuperState.snowBlowerScore);
            }else
                setSuperState(SuperState.score);
            return true;
        }
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }
        return false;
    }

    private boolean score(){
        targeting.setToggle(true);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(scoringState);
        }
        if(targeting.shotPossible()) {
            shooter.setState(ShooterState.followAimbot);
        } else {
            shooter.setState(ShooterState.followAimbot);
        }
        swerve.setSnapAngle(targeting.getCalculatedDirection());
        intake.setState(IntakeState.gyrate);
        hopper.setState(HopperState.shoot);

        if(intaking) {
            setSuperState(SuperState.snowBlowerScore);
        }
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }

        return shooter.isAtSetpoint();
    }

    private boolean snowBlowerScore(){
        targeting.setToggle(true);
        intake.setState(IntakeState.intake);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(scoringState);
        }
        swerve.setSnapAngle(targeting.getCalculatedDirection());
        shooter.setState(ShooterState.followAimbot);
        hopper.setState(HopperState.shoot);
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }
        if(!intaking) {
            setSuperState(SuperState.score);
        }
        
        return shooter.isAtSetpoint();
    }

    private boolean manual(){
        // code for direct control of mechanisms goes here
        targeting.setToggle(true);
        intake.setState(IntakeState.manual);
        targeting.setShotType(shotType.Manual);
        hopper.setState(HopperState.manual);
        shooter.setState(ShooterState.followAimbot);
        
        /*if(!operatorController.getAButton()){
            intake.setPosition(Math.abs(operatorController.getRightY() * 128));
        }else
            intake.setPosition(128);
        */
        //hopper.setSpeed(operatorController.getLeftTriggerAxis());

        return true;
    }

    /** 
     * The state of the robot when autonomous initializes.
     * <p>It will reset/initialize certain variables and set the auto to the selected list of {@link AutoAction}'s from the auto chooser in {@link Autonomous}
     * before switching to the autonomous state. It also calls the autonomous state so it does not have to wait an extra iteration
     * @return true since there is no logic to compute
     */
    private boolean autonomousEnter(){
        setSuperState(SuperState.autonomous);
        autoIndex = 0;
        autonomous();
        return true;
    }

    /**
     * The state of the robot during the full autonomous period.
     * This will run each {@link AutoAction} from the list received from {@link Autonomous} in order.
     * <p>If there is an alternative condition in the AutoAction, this method should increment the autoIndex variable after the condition is true.
     * <p>Otherwise, if the AutoAction is a {@link Path}, this method should increment the variable when the path is finished;
     * if it's a {@link SuperState}, this method should increment the variable when the SuperState's state method returns true 
     * @return true if autoIndex >= the size of the AutoAction List
     */
    private boolean autonomous(){
        inAuto = true;
        boolean isActionFinished = true;
        AutoAction autoAction;
        autoAction = autoActions.get(autoIndex);

        SmartDashboard.putNumber("AutoAction", autoActions.size() - 1);
        SmartDashboard.putNumber("autoIndex", autoIndex);
        SmartDashboard.putBoolean("IsPathFinished", isActionFinished);

        if (autoAction.isPath()){
            SwerveState swerveState = swerve.getSwerveState();
            if (!swerveState.equals(SwerveState.initializePath) && !swerveState.equals(SwerveState.runPath)){
                swerve.setPath(autoAction.getAsPath());
                swerve.setSwerveState(SwerveState.initializePath);
            }
            isActionFinished = autoAction.getAsPath().isPathFinished();
        } else if (autoAction.isState()) {
            isActionFinished = manageSuperState(autoAction.getAsState());
        }

        if (autoAction.hasAlternateCondition()){
            isActionFinished = autoAction.getAlternateCondition();
        }

        if (autoAction.hasAdditionalCondition()){
            isActionFinished = (isActionFinished && autoAction.getAdditionalCondition());
        }

        if (autoAction.hasOptionalCondition()){
            isActionFinished = (isActionFinished || autoAction.getOptionalCondition());
        }

        if (isActionFinished) {
            if (autoActions.size() - 1 > autoIndex) {
                autoIndex += 1;
                
            }
            else {
                inAuto = false;
                setSuperState(SuperState.autonomousExit);
            }

            
        }

        return true;
    }

    /**
     * The state of the robot at the end of the autonomous period.
     * <p>This will set the robot up for Teleop control by setting inAuto to false, 
     * setting the swerve state to manual, and changing the superState (using setSuperState()) to a different state
     * @return true 
     */
    private boolean autonomousExit(){
        inAuto = false;
        swerve.setSwerveState(SwerveState.manual);
        setSuperState(SuperState.deployed);
        
        //TODO: Code for setting up teleop goes here.
        // this may include setting swerve to manual control, setting the superState to a different state, etc.
        return true;
    }

    /**
     * The method called when the state method of the managed SuperState is not called.
     * @param state
     * @return
     */
    private boolean defaultState(SuperState state){
        System.err.println("The SuperState \"" + state.name() + "\" does not have a state method or it was not called."
            + "\nPlease return the state method in a new case statement in manageState()");
        return true;
    }

    /**
     * This method is responsible for changing the robot's {@link SuperState}.
     * <p> This method MUST be used when changing the robot's SuperState to ensure it does not interfere with autonomous.
     * @param state the SuperState to set the robot if not in Autonomous mode
     */
    public void setSuperState(SuperState state){
    
        if(!inAuto)
            superState = state;
    }

    /**
     * Gets the current {@link SuperState} of the robot.
     * @return the value of the superState object
     */
    public SuperState getSuperState(){
        return superState;
    }

    public void disableAuto(){
        superState = SuperState.autonomousExit;
    }

    
}
