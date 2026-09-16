package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.Aimbot.presetShotType;
import team7111.robot.subsystems.Aimbot.shotType;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private Timer timer = new Timer();

    private Field zone;

    private StructArrayPublisher<Pose2d> hubPublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Hub Presets", Pose2d.struct).publish();

    private WaypointConstraints fastTransConstraints = new WaypointConstraints(8, 0, 6, 1);
    private WaypointConstraints fastRotConstraints = new WaypointConstraints(720, 0, 180, 90);
    
    private WaypointConstraints balancedTransConstraints = new WaypointConstraints(6, 2, 4, 0.4);
    private WaypointConstraints balancedRotConstraints = new WaypointConstraints(270, 0, 90, 5);

    private WaypointConstraints slowTransConstraints = new WaypointConstraints(1, 0, 2, 0.2);
    private WaypointConstraints slowRotConstraints = new WaypointConstraints(180, 0, 45, 0.8);

    private SendableChooser<Autos> autoChooser = new SendableChooser<>();
    private SuperStructure superStructure;

    private Pose2d assumedStartingPose = new Pose2d(3.5, 4, Rotation2d.kZero);

    private final Waypoint[] trenchLWaypoints = new Waypoint[]{
        new Waypoint(new Pose2d(3.8, 7.446, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
        new Waypoint(new Pose2d(5.9, 7.446, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
    };
    private final Waypoint[] trenchRWaypoints = new Waypoint[]{
        new Waypoint(new Pose2d(3.8, 0.675, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
        new Waypoint(new Pose2d(5.9, 0.675, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
    };

    public final Pose2d[] hubPresetPoses = new Pose2d[]{
        // TODO get coordinates for poses near trench
        new Pose2d(4.15, 0.625, Rotation2d.fromDegrees(90)),
        new Pose2d(4.5, 7.45, Rotation2d.fromDegrees(-90)),
        new Pose2d(2.996, 5.946, Rotation2d.fromDegrees(-45.65)),
        new Pose2d(2.267, 4.021, Rotation2d.fromDegrees(0)),
        new Pose2d(2.996, 2.096, Rotation2d.fromDegrees(52.65)),
        new Pose2d(0.85, 7, Rotation2d.fromDegrees(-38.5)),
        new Pose2d(0.85, 1.16, Rotation2d.fromDegrees(38.5)),
    };

    private final Pose2d lTrenchStartPose = new Pose2d(4.25, 7.5, Rotation2d.fromDegrees(-90));
    private final Pose2d rTrenchStartPose = new Pose2d(4.25, 0.65, Rotation2d.fromDegrees(-90));
    private final Pose2d hubStartPose =     new Pose2d(3.75, 4, Rotation2d.kZero);
    //starting position abreviated. separated with "_"
    
    /**
     * {@link Autonomous#getAutonomous(Autos)}
     */
    public enum Autos {
        forwardTest,
        
        rt_Bump,
        lt_Bump,
        rt_Trench,
        lt_Trench,

        rt_TrenchTrench, 
        lt_TrenchTrench, 
        rt_TrenchOutpost,
        lt_TrenchDepot,

        shoot,
        shootDepot,
        shootOutpost, 
        backupShoot,
    }

    /**
     * {@link Autonomous#getPath(Paths)}
     */
    public enum Paths {
        forward,
        forwardR,

        sideSwipeR,
        sideSwipeL,
        outpost,
        depot,

        hubSetpointL,
        hubSetpointM,
        hubSetpointR,
        hubSetpointRT,
        hubSetpointLT,
        trenchLNeutral,
        trenchRNeutral,
        trenchLAlliance,
        trenchRAlliance,

        RNsweep,
        LNsweep,

        RSweepToBump,
        LSweepToBump,
        
        RSweepToTrench,
        LSweepToTrench, 

        hubMiddle, 
        depotLTrench,
    }

    public Autonomous(Field zone){
        Autos defaultAuto;
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        defaultAuto = Autos.shoot;
        //defaultAuto = Autos.lt_Trench;
        //defaultAuto = Autos.rt_Trench;
        //defaultAuto = Autos.shootDepot;
        
        autoChooser.setDefaultOption(defaultAuto.name(), defaultAuto);
        this.zone = zone;
        
        Shuffleboard.getTab("Autonomous").add("AutoChooser", autoChooser);
        Shuffleboard.getTab("Autonomous").addString("Selected Auto", () -> getSelectedAuto().name());

        hubPublisher.accept(hubPresetPoses);

    }

    public void periodic(){
        
    }

    public void simulationPeriodic(){}

    /**
     * 
     * @param autoName
     * @return
     * {@link Autos}
     */
    public ArrayList<AutoAction> getAutonomous(Autos autoName){
        ArrayList<AutoAction> auto = new ArrayList<>();
        // define each autonomous using a switch statement.
        // each auto is an array of "AutoAction's"
        switch (autoName) {
            case forwardTest:
                auto.add(new AutoAction(getPath(Paths.forward)));
                    
                break;

            case rt_TrenchOutpost:
                auto = getAutonomous(Autos.rt_Trench);
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.outpost)));
                auto.add(new AutoAction(SuperState.deployed).withNoConditions());
                auto.add(new AutoAction(getNearestHubScoringPath(hubPresetPoses[0])));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(10);
                }));
                break;
            
            case lt_TrenchDepot:
                auto = getAutonomous(Autos.lt_Trench);
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.depotLTrench)));
                auto.add(new AutoAction(getPath(Paths.hubSetpointL)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(10);
                }));
                break;
            
            case lt_Bump:
                assumedStartingPose = lTrenchStartPose;
                auto.add(new AutoAction(SuperState.intake));
                auto.add(new AutoAction(getPath(Paths.LNsweep)));
                auto.add(new AutoAction(SuperState.deployed));
                auto.add(new AutoAction(getPath(Paths.LSweepToBump)));
                auto.add(new AutoAction(getPath(Paths.hubSetpointL)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(10);
                }));
                break;
            case rt_Bump:
                assumedStartingPose = rTrenchStartPose;
                auto.add(new AutoAction(SuperState.intake));
                auto.add(new AutoAction(getPath(Paths.RNsweep)));
                auto.add(new AutoAction(SuperState.deployed));
                auto.add(new AutoAction(getPath(Paths.RSweepToBump)));
                auto.add(new AutoAction(getPath(Paths.hubSetpointR)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(10);
                }));
                break;
            case lt_Trench:
                assumedStartingPose = lTrenchStartPose;
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.LNsweep)));
                auto.add(new AutoAction(getPath(Paths.LSweepToTrench)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(15);
                }));

                break;
            case rt_Trench:
                assumedStartingPose = rTrenchStartPose;
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.RNsweep)));
                auto.add(new AutoAction(getPath(Paths.RSweepToTrench)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(15);
                }));
                break;
            case lt_TrenchTrench:
                auto = getAutonomous(Autos.lt_Trench);
                auto.add(new AutoAction(SuperState.intake).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Transport); 
                    return true;}));
                auto.add(new AutoAction(getPath(Paths.sideSwipeL)));
                auto.add(new AutoAction(getPath(Paths.trenchLAlliance).withRotation(-90)));
                auto.add(new AutoAction(SuperState.preparePass).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return timeDelay(5);
                }));
                break;
            case rt_TrenchTrench:
                auto = getAutonomous(Autos.rt_Trench);
                auto.add(new AutoAction(SuperState.intake).withAlternateCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Transport); 
                    return true;}));
                auto.add(new AutoAction(getPath(Paths.sideSwipeR)));
                auto.add(new AutoAction(getPath(Paths.trenchRAlliance).withRotation(90)));
                auto.add(new AutoAction(SuperState.preparePass).withAlternateCondition(() -> {
                    superStructure.targeting.setPreset(presetShotType.Trench);
                    return timeDelay(5);
                }));
                break;
            case shoot:
                assumedStartingPose = hubStartPose;
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                break;
            case shootDepot:
                assumedStartingPose = hubStartPose;
                auto.add(new AutoAction(SuperState.prepareHubShot).withAdditionalCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return true;
                }));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                auto.add(new AutoAction(SuperState.intake));
                auto.add(new AutoAction(getPath(Paths.depot)).withOptionalCondition(() -> timeDelay(5)));
                auto.add(new AutoAction(getPath(Paths.hubSetpointM)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                break;
            case shootOutpost:
                assumedStartingPose = hubStartPose;
                auto.add(new AutoAction(SuperState.prepareHubShot).withAdditionalCondition(() -> {
                    superStructure.targeting.setShotType(shotType.Parabolic);
                    return true;
                }));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                auto.add(new AutoAction(SuperState.deployed));
                auto.add(new AutoAction(getPath(Paths.outpost)));
                auto.add(new AutoAction(SuperState.deployed).withAlternateCondition(() -> {return timeDelay(5);}));
                auto.add(new AutoAction(getPath(Paths.hubSetpointM)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                break;
            case backupShoot:
                auto = getAutonomous(Autos.shoot);
                auto.add(new AutoAction(getPath(Paths.hubSetpointM).withRotation(-90)));
                auto.add(new AutoAction(SuperState.deployed));
                break;
            default:
                break;
        }

        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get().equals(Alliance.Red)){
                assumedStartingPose = new Pose2d(16.5354 - assumedStartingPose.getX(), 8.001 - assumedStartingPose.getY(), Rotation2d.kZero);
            }
        }
        return auto;
    }

    /**
     * 
     * @param path
     * @return
     * {@link Paths}
     */
    public Path getPath(Paths path){
        List<Waypoint> waypoints = new ArrayList<>();
        // define Path object for each Paths enum using a switch statement
        switch (path) {
            case forward:
                waypoints.add(balancedPoint(5, 0, 0));
                waypoints.add(balancedPoint(5, 5, 0));
                
                break;
            case forwardR:
                waypoints.add(balancedPoint(6, 0.625, 180));
                break;
            case sideSwipeR:
                waypoints.add(balancedPoint(5.985, 0.96, 90));
                waypoints.add(slowPoint(5.985, 3.3, 0));
                break;
            case sideSwipeL:
                waypoints.add(balancedPoint(5.985, 7.09072, -90));
                waypoints.add(slowPoint(5.985, 4.7, -180));
                break;
            case hubSetpointRT:
                waypoints.add(balancedPoint(4.25, 0.625, 90));
                break;
            case hubSetpointLT:
                waypoints.add(balancedPoint(4.25, 7.45, -90));
                break;
            case hubSetpointL:
                waypoints.add(balancedPoint(hubPresetPoses[2].getX(), hubPresetPoses[2].getY(), hubPresetPoses[2].getRotation().getDegrees()));
                break;
            case hubSetpointM:
                waypoints.add(balancedPoint(hubPresetPoses[3].getX(), hubPresetPoses[3].getY(), hubPresetPoses[3].getRotation().getDegrees()));
                break;
            case hubSetpointR:
                waypoints.add(balancedPoint(hubPresetPoses[4].getX(), hubPresetPoses[4].getY(), hubPresetPoses[4].getRotation().getDegrees()));
                break;
            case trenchLAlliance:
                waypoints.add(trenchLWaypoints[1]);
                waypoints.add(trenchLWaypoints[0]);
                break;
            case trenchLNeutral:
                waypoints.add(trenchLWaypoints[0]);
                waypoints.add(trenchLWaypoints[1]);
                break;
            case trenchRAlliance:
                waypoints.add(trenchRWaypoints[1]);
                waypoints.add(trenchRWaypoints[0]);
                break;
            case trenchRNeutral:
                waypoints.add(trenchRWaypoints[0]);
                waypoints.add(trenchRWaypoints[1]);
                break;
            case RNsweep:
                waypoints.add(fastPoint(6.1, 0.68, -90));
                waypoints.add(balancedPoint(7.7, 1.26, 0));
                waypoints.add(slowPoint(7.6, 3.5, 0));
                break;
            case LNsweep:
                waypoints.add(fastPoint(6.1, 7.389072, -90));
                waypoints.add(fastPoint(7.7, 6.809072, 180));
                waypoints.add(fastPoint(7.6, 5.00569072, 180));
                break;
            case RSweepToBump:
                waypoints.add(fastPoint(6.3, 2.5, -135));
                waypoints.add(balancedPoint(2.26, 2.46, -135));
                break;
            case LSweepToBump:
                waypoints.add(slowPoint(6.3, 5.00669072, -135));
                waypoints.add(balancedPoint(2.26, 5.689072, -135));
                break;
            case RSweepToTrench:
                //waypoints.add(fastPoint(6.334, 3.071, 127.5)); // temp uncomment
                //waypoints.add(balancedPoint(5.86, 1.86, 179)); // temp uncomment

                //waypoints.add(balancedPoint(5.86, 1.86, 90)); // uncomment if running into wall
                waypoints.add(balancedPoint(5.86, 0.68, 90));
                waypoints.add(balancedPoint(3.94, 0.68, 90));
                break;
            case LSweepToTrench:
                //waypoints.add(fastPoint(6.334, 4.4, 63)); // temp uncomment
                //waypoints.add(balancedPoint(5.86, 6, 0)); // temp uncomment

                //waypoints.add(balancedPoint(5.86, 6, -90)); // uncomment if running into wall
                waypoints.add(balancedPoint(5.86, 7.389072, -90));
                waypoints.add(balancedPoint(3.94, 7.389072, -90));
                break;
            case outpost:
                waypoints.add(balancedPoint(1.5, 0.7, 90));
                waypoints.add(slowPoint(0.76, 0.7, 90));
                break;
            case depot:
                waypoints.add(balancedPoint(2.376, 4.03, 0));

            case depotLTrench:
                waypoints.add(balancedPoint(1.78, 5.78, 90));
                waypoints.add(slowPoint(0.7, 5.85, 90));
                break;
            case hubMiddle:
                waypoints.add(balancedPoint(0, 0, 0));

            default:
                break;
        }
        return new Path(waypoints);
    }

    public Waypoint fastPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), fastTransConstraints, fastRotConstraints);
    }

    public Waypoint balancedPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), balancedTransConstraints, balancedRotConstraints);
    }

    public Waypoint slowPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), slowTransConstraints, slowRotConstraints);
    }

    public Autos getSelectedAuto(){
        return autoChooser.getSelected();
    }

    public Path getNearestHubScoringPath(Pose2d robotPose){
        //TODO this function will return the nearest Pose2d to robotPose from hubPresetPoses on line 45.
        // If it is one of the trench poses (one of the first 2 poses in the array), it will check if it is closest to
        // that same pose, the pose with 1.5 x added, or the pose with 1.5 x subtracted.
        // Use the nearest() method in the Pose2d class to find the nearest pose.
        List<Waypoint> waypoints = new ArrayList<>();
        List<Pose2d> hubPoses = new ArrayList<>();
        double mapLengthX = 16.5354;
        double mapLengthY = 8.001;
        Pose2d ogRobotPose = robotPose;

        if (zone.inAllianceZone(robotPose)) {
            for (Pose2d pose: hubPresetPoses) {
                hubPoses.add(pose);
            }
        } else {
            hubPoses.add(hubPresetPoses[0]);
            hubPoses.add(hubPresetPoses[1]);
        }

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                robotPose = new Pose2d(-robotPose.getX() + mapLengthX, -robotPose.getY() + mapLengthY, Rotation2d.fromDegrees(((robotPose.getRotation().getDegrees() +180) * 1) % 180));
            }
        }
        Pose2d nearestPose = robotPose.nearest(hubPoses);
        double robotPoseX = nearestPose.getX();
        double robotPoseY = nearestPose.getY();
        double robotPoseRot = nearestPose.getRotation().getDegrees();
        double flipMult = 1.0;
        boolean flipBool = zone.inAllianceZone(ogRobotPose);
        double allianceOffset = 1;
        double neutralOffset = 1.5;

        if ((robotPoseX == hubPresetPoses[1].getX() && robotPoseY == hubPresetPoses[1].getY()) || (robotPoseX == hubPresetPoses[0].getX() && robotPoseY == hubPresetPoses[0].getY())) {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    flipBool = !flipBool;
                    flipMult = -1.0;
                    allianceOffset = neutralOffset;
                    neutralOffset = 1;
                }
                if (flipBool) {
                    waypoints.add(balancedPoint(robotPoseX - (allianceOffset * flipMult), robotPoseY, robotPoseRot));
                } else {
                    waypoints.add(balancedPoint(robotPoseX + (neutralOffset * flipMult), robotPoseY, robotPoseRot));
                }
                
            }
    }
        waypoints.add(new Waypoint(nearestPose, balancedTransConstraints, balancedRotConstraints));
        return new Path(waypoints);
    }

    public Path getNearestTrenchPath(Pose2d robotPose){
        List<Path> trenchPaths = new ArrayList<>();
        List<Pose2d> trenchPoses = new ArrayList<>();
        if(robotPose.getX() > 4.7 && robotPose.getX() < 11.957){
            trenchPaths.add(getPath(Paths.trenchLAlliance));
            trenchPaths.add(getPath(Paths.trenchRAlliance));
        }else{
            trenchPaths.add(getPath(Paths.trenchLNeutral));
            trenchPaths.add(getPath(Paths.trenchRNeutral));
        }

        for (Path path : trenchPaths) {
            trenchPoses.add(path.getCurrentWaypoint().getPose());
        }

        Pose2d pose = robotPose.nearest(trenchPoses);

        for (Path path : trenchPaths) {
            if(pose.getX() == path.getCurrentWaypoint().getPose().getX()
             && pose.getY() == path.getCurrentWaypoint().getPose().getY()){
                return path;
            }
        }
        return null;
    }

    public void giveResources(SuperStructure superStructure){
        this.superStructure = superStructure;
    }

    public Pose2d getAssumedPose(){
        return assumedStartingPose;
    }

    private boolean timeDelay(int delay){
        timer.start();
        if (timer.hasElapsed(delay)) {
            timer.reset();
            timer.stop();
            return true;
        }
        return false;
    }
}
