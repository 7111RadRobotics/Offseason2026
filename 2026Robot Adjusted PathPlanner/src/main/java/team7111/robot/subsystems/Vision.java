package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import team7111.robot.utils.Camera;

public class Vision extends SubsystemBase{
    //we on longer have a limelight on the robot, however we may one day need to put it back on again. Therefore, I have left this code inside of the program, although it may make it less readable, it could be useful one day. Thank you for taking the time to read this wonderful message and I hope you have a great day :D
    //private PhotonCamera camera1 = new PhotonCamera("photonvision1");

    /**
     * Length is the number of cameras.
     * Each index is the position of the camera.
     * Add new cameras by extending the array
     */
    private final Transform3d cameraPositionsToCenter[] = {
        new Transform3d(Units.inchesToMeters(-3.538), Units.inchesToMeters(2.714), Units.inchesToMeters(19.197), new Rotation3d(Degrees.of(180).in(Radians), Degrees.of(25).in(Radians), 0)),
        new Transform3d(Units.inchesToMeters(-12.527), Units.inchesToMeters(-10.395), Units.inchesToMeters(6.575), new Rotation3d(0, Degrees.of(15).in(Radians), Degrees.of(-165).in(Radians) )),
    };

    //private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();

    private List<PhotonTrackedTarget> targets = new ArrayList<>();

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator poseEstimator;

    // TODO: change variable names on actual robot
    /*public final Camera limelight = new Camera(
        "photonvision", 
        Constants.vision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );*/
    public final Camera shooterCam;
    public final Camera climberCam;
    public final Camera intakeCam;

    public Camera[] cameraList;

    private double allowedPoseAmbiguity = 1;
    private double objectDetectionYaw = 0;

    /** Constructor */
    public Vision(){
        CameraServer.startAutomaticCapture(0);

        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (UncheckedIOException e) {
            throw new RuntimeException(e);
        }
        poseEstimator = new PhotonPoseEstimator(fieldLayout, Transform3d.kZero);
        
        targets.add(new PhotonTrackedTarget());

        shooterCam = new Camera(
            "OV9281_4", 
            cameraPositionsToCenter[0], 
            new EstimatedRobotPose(estPose3d, 0.0, targets, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR), 
            this
        );

        climberCam = new Camera(
            "OV9281_3 (1)", 
            cameraPositionsToCenter[1], 
            new EstimatedRobotPose(estPose3d, 0.0, targets, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR), 
            this
        );

        intakeCam = new Camera(
            "Microsoft_LifeCam_HD-3000", 
            cameraPositionsToCenter[0], 
            new EstimatedRobotPose(estPose3d, 0.0, targets, PoseStrategy.AVERAGE_BEST_TARGETS), 
            this
        );


        cameraList = new Camera[] {
            shooterCam,
            climberCam,
        };
        
    }

    public void periodic(){

        Optional<EstimatedRobotPose> estPose;
        PhotonPipelineResult objectResult = intakeCam.getLatestResult();
        if(objectResult != null){
            if(objectResult.hasTargets()){
                objectDetectionYaw = objectResult.getBestTarget().getYaw();
            }
        }
        for(Camera camera : cameraList){
            estPose = camera.getEstimatedGlobalPose(robotPose);
            robotPose = camera.estRobotPose.estimatedPose.toPose2d();
            if(estPose.isPresent()){
                if(estPose.get() != null)
                    camera.estRobotPose = estPose.get();
            }

            camera.periodic();
        }
    }

    /**
     * returns the position of the robot averaged between all cameras
     */
    public Pose3d getRobotPose(double ambiguityThreshold) {
        
        boolean hasPose = false;
        double averageX = 0;
        double averageY = 0;
        double averageZ = 0;
        double averageRot = 0;
        int numOfCamerasSeen = 0;
        for(int i = 0; i < cameraList.length; i++) {
            PhotonPipelineResult result = cameraList[i].getLatestResult();
            if(result.hasTargets()) {
                if(result.getBestTarget().getPoseAmbiguity() > ambiguityThreshold){
                    hasPose = true;
                    averageX += cameraList[i].estRobotPose.estimatedPose.getX();
                    averageY += cameraList[i].estRobotPose.estimatedPose.getY();
                    averageZ += cameraList[i].estRobotPose.estimatedPose.getZ();
                    averageRot += cameraList[i].estRobotPose.estimatedPose.getRotation().getAngle();
                    numOfCamerasSeen++;
                }
            }
        }

        if(!hasPose) {
            return null;
        }

        averageX /= numOfCamerasSeen;
        averageY /= numOfCamerasSeen;
        averageZ /= numOfCamerasSeen;
        averageRot /= numOfCamerasSeen;

        Pose3d estPose = new Pose3d(averageX, averageY, averageZ, new Rotation3d(new Rotation2d(averageRot)));

        return hasPose 
            ? estPose
            : null;
    }

    /**
     * Returns null if the apriltag is not found.
     */
    public Pose3d getRobotPose(int apriltag) {
        int[] targetId = new int[cameraList.length];

        for(int i = 0; i < cameraList.length; i++) {
            for(int j = 0; j < cameraList[0].getLatestResult().targets.size(); j++) {
                if(cameraList[i].getLatestResult().getTargets().get(j).fiducialId == apriltag) {
                    targetId[i] = j;
                    break;
                } else {
                    targetId[i] = -1;
                }
            }
        }
        
        
        Transform3d transform = null;
        for(int i = 0; i < cameraList.length; i++) {
            if(targetId[i] != -1) {
                transform = cameraList[i].getLatestResult().getTargets().get(targetId[i]).getBestCameraToTarget();
                break;
            }
        }

        //If the target is not found, returns null
        if(transform == null) {
            return null;
        }

        Pose3d estPose = cameraList[0].getApriltagPos(apriltag).transformBy(transform);
        
        
        return estPose;
    }

    public Pose3d getBestRobotPose(double ambiguityThreshold){
        double[] bestAmbiguities = new double[cameraList.length];
        for (int i = 0; i < cameraList.length; i++) {
            bestAmbiguities[i] = 1.0;
            Camera camera = cameraList[i];
            if(camera.getLatestResult().hasTargets()){
                for(PhotonTrackedTarget target : camera.estRobotPose.targetsUsed){
                    if(target.poseAmbiguity < bestAmbiguities[i]){
                        bestAmbiguities[i] = target.poseAmbiguity;
                    }
                }
            }
        }
        Pose3d robotPose = null;
        double bestAmbiguity = ambiguityThreshold;
        for (int i = 0; i < bestAmbiguities.length; i++) {
            if(bestAmbiguities[i] <= bestAmbiguity){
                bestAmbiguity = bestAmbiguities[i];
                robotPose = cameraList[i].estRobotPose.estimatedPose;
            }
        }

        if(robotPose != null){
            return robotPose;
        }
        return null;
    }

    /** Gets the robot position from a specific camera*/
    public Pose3d getRobotPose(Camera camera, double ambiguityThreshold) {
        if (camera.getLatestResult().hasTargets()) {
            double bestAmbiguity = 1.0;
            for(PhotonTrackedTarget target : camera.estRobotPose.targetsUsed){
                if(target.poseAmbiguity < bestAmbiguity){
                    bestAmbiguity = target.poseAmbiguity;
                }
            }
            if(bestAmbiguity > ambiguityThreshold){
                return null;
            }
            return camera.estRobotPose.estimatedPose;
        }
        return null;
    }

    public double getGamepieceYaw(){
        return objectDetectionYaw;
    }
}
