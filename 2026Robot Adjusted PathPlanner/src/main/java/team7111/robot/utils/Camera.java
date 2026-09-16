package team7111.robot.utils;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.List;
import java.util.Optional;
import java.util.Vector;

import team7111.robot.subsystems.Vision;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

public class Camera extends PhotonCamera{
    private Transform3d cameraToRobotCenter;
    private Vision vision;
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    private PhotonTrackedTarget bestTarget;
    private Transform3d bestCameraToTarget = new Transform3d();
    public EstimatedRobotPose estRobotPose;
    public int bestTargetId;
    private AprilTagFieldLayout apriltagMap;
    private Pose2d newPose = new Pose2d();
    private boolean isObjectCam = false;
    {
        try {
            apriltagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
        } catch (IOException e){
            throw new RuntimeException(e);
        }
    }

    final int fuelObjID = -1;

    //From 2025 on season code
    private final Matrix<N3,N1> visionStandardDeviation = MatBuilder.fill(Nat.N3(), Nat.N1(),1,1,1 * Math.PI);
    private final double noisyDistanceMeters = 2.5;
    private final double distanceWeight = 7;
    private final double tagPresenceWeight = 10;
    private final double poseAmbiguityShifter = 0.2;
    private final double poseAmbiguityMultiplier = 4;

    public Camera(String cameraName, Transform3d cameraToRobotCenter, EstimatedRobotPose estRobotPose, Vision vision) {
        super(cameraName);
        this.cameraToRobotCenter = cameraToRobotCenter;
        this.vision = vision;
        this.estRobotPose = estRobotPose;
        photonPoseEstimator = new PhotonPoseEstimator(apriltagMap, estRobotPose.strategy, cameraToRobotCenter); //WAS ERRORED OUT
    }

    public Camera(String cameraName){
        super(cameraName);
        isObjectCam = true;
    }
    
    public void periodic(){
        //Gets the latest result of the unread results.
        List<PhotonPipelineResult> unreadResults = getAllUnreadResults();
        if(!unreadResults.isEmpty()){
            for (PhotonPipelineResult result : unreadResults){
                latestResult = result;
            }
            unreadResults.clear();
        }
        
        
        if(latestResult.hasTargets()){
            bestTarget = latestResult.getBestTarget();
            bestCameraToTarget = bestTarget.getBestCameraToTarget();
            bestTargetId = bestTarget.getFiducialId();
        }
        if(!isObjectCam){
            Optional<EstimatedRobotPose> estPose = getEstimatedGlobalPose(vision.robotPose);
            if(estPose.isPresent()){
                if(photonPoseEstimator.getRobotToCameraTransform() != cameraToRobotCenter){
                    photonPoseEstimator.setRobotToCameraTransform(cameraToRobotCenter);
                }
                estRobotPose = estPose.get();
            }
        }
        
    }

    public boolean updatePose(){
        return latestResult.hasTargets();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(latestResult);
    }

    public Pose2d getRobotPose(){
        newPose = estRobotPose.estimatedPose.toPose2d();
        return newPose;
    }

    public Matrix<N3, N1> getPoseAmbiguity(){
        double smallestDistance = Double.POSITIVE_INFINITY;
        double confidenceMultiplier = 0;
        if(estRobotPose != null){
            for (var target : estRobotPose.targetsUsed) {
                var t3d = target.getBestCameraToTarget();
                var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                }
            }
            double poseAmbiguityFactor = estRobotPose.targetsUsed.size() != 1
                ? 1
                : Math.max(1, estRobotPose.targetsUsed.get(0).getPoseAmbiguity() + poseAmbiguityShifter * poseAmbiguityMultiplier);
            confidenceMultiplier = Math.max(1,
                (Math.max(1, Math.max(0, smallestDistance - noisyDistanceMeters) * distanceWeight) * poseAmbiguityFactor) 
                / (1 + ((estRobotPose.targetsUsed.size() - 1) * tagPresenceWeight)));
        }
        SmartDashboard.putNumber(getName(), confidenceMultiplier);
        return visionStandardDeviation.times(confidenceMultiplier);
    }

    public Transform3d getCameraToRobot(){
        return cameraToRobotCenter;
    }

    public PhotonPipelineResult getLatestResult(){
        if(latestResult != null)
            return latestResult;
        return new PhotonPipelineResult();
    }

    /**
     * Gets the distance from the camera to the target
     */
    public Transform3d getCamToTarget() {
        return bestCameraToTarget;
    }

    public Pose3d getApriltagPos(int apriltag) {

        return apriltagMap.getTagPose(apriltag).get();
    }

    /**Gets all targets of the type specified as "fuelObjID" and returns in a vector :)*/
    public Vector<PhotonTrackedTarget> objectDetection() {
        Vector<PhotonTrackedTarget> targets = new Vector<>(0);
        for(int i = 0; i < this.getLatestResult().getTargets().size(); i++) {
            if(this.getLatestResult().getTargets().get(i).getDetectedObjectClassID() == fuelObjID) {
                targets.add(this.getLatestResult().getTargets().get(i));
            }
        }
        return targets;
    }

    public boolean isObjectCam(){
        return isObjectCam;
    }
}

