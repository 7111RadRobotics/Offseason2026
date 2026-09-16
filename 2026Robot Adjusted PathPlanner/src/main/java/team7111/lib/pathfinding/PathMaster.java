package team7111.lib.pathfinding;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PathMaster {
    private PIDController xPID;
    private PIDController yPID;
    private PIDController rotPID;

    private Supplier<Pose2d> suppliedPose;
    private Supplier<Rotation2d> gyroYaw;

    private double xCalculation;
    private double yCalculation;
    private double rotCalculation;

    private double invertedX = 1.0;
    private double invertedY = 1.0;
    private double invertedRot = 1.0;
    private double invertedGyro = 1.0;

    //Houses field elements
    private FieldElement fieldElements[];

    private boolean shouldFlipPath = false;
    private boolean isPathMirrored = false;
    private boolean fieldRelative = false;

    private boolean avoidFieldElements = false;
    
    public PathMaster(Supplier<Pose2d> suppliedPose, Supplier<Rotation2d> gyroYaw){

        xPID = new PIDController(1, 0, 0);
        yPID = new PIDController(1, 0, 0);

        this.rotPID = new PIDController(1, 0, 0);
        rotPID.enableContinuousInput(-180, 180);
        this.suppliedPose = suppliedPose;
        this.gyroYaw = gyroYaw;
    }
    
    /**
     * Determines if the path should be flipped or not.
     * @param flipField -Which team it is on. False = alliance 1, true = alliance 2
     * @param isMirrored -Determines if the path should be flipped. If false, it will not flip.
     */
    public void useAllianceFlipping(boolean flipField, boolean isMirrored){
        shouldFlipPath = flipField;
        isPathMirrored = isMirrored;
    }
    
    public void useFieldRelative(boolean isFieldRelative){
        fieldRelative = isFieldRelative;
    }

    /**
     * Sets the translation pid for the path
     */
    public void setTranslationPID(double P, double I, double D){
        xPID.setPID(P, I, D);
        yPID.setPID(P, I, D);
    }

    /**
     * Sets the rotation pid for the path
     */
    public void setRotationPID(double P, double I, double D){
        rotPID.setPID(P, I, D);
    }

    /**
     * Sets the field element map to set where the objects are
     */
    public void setFieldElementMap(FieldElement[] fieldElementArray){
        fieldElements = fieldElementArray;
    }

    /**
     * Sets the inversions for the robot
     */
    public void setInversions(boolean invertX, boolean invertY, boolean invertRot, boolean invertGyro){
        invertedX = 1.0;
        invertedY = 1.0;
        invertedRot = 1.0;
        invertedGyro = 1.0;

        if (invertX){
            invertedX = -1.0;
        }

        if (invertY){
            invertedY = -1.0;
        }

        if (invertRot){
            invertedRot = -1.0;
        }

        if (invertGyro){
            invertedGyro = -1.0;
        }
    }

    
    void useBrokenPathFinding(boolean avoidFieldElements){
        this.avoidFieldElements = avoidFieldElements;
    }

    
    /**
     * Equivelant to a "Reset" command, it applies all setup methods and initilizes the path
     */
    public void initializePath(Path path){
        path.setPoseSupplier(suppliedPose);
        path.setSpeedSuppliers(()-> xCalculation, ()-> yCalculation, ()-> rotCalculation);
        path.flipPath(shouldFlipPath, isPathMirrored);
        path.avoidFieldElements(avoidFieldElements, fieldElements);
        path.initialize();
    }

    /**
     * Runs the path's periodic and calculates the speed suppliers for x, y, and rotation.
     */
    public void periodic(Path path){
        
        path.periodic();
        xCalculation = xPID.calculate(suppliedPose.get().getX(), path.getCurrentWaypoint().getPose().getX()) * invertedX;
        yCalculation = yPID.calculate(suppliedPose.get().getY(), path.getCurrentWaypoint().getPose().getY()) * invertedY;
        rotCalculation = rotPID.calculate(
                            suppliedPose.get().getRotation().getDegrees(), 
                            path.getCurrentWaypoint().getPose().getRotation().getDegrees()) * invertedRot;

    }

    /**
     * Returns a ChassisSpeeds object for handing to the swerve subsystem
     */
    public ChassisSpeeds getPathSpeeds(Path path, boolean avoidFieldElements, boolean fieldRelative){

        if (path.firstSpeed) {
            return path.getSpeeds();
        } else {
        
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(path.getChassisSpeeds().vxMetersPerSecond, path.getChassisSpeeds().vyMetersPerSecond, path.getRotationSpeed(), gyroYaw.get().times(invertedGyro))
            : new ChassisSpeeds(path.getTranslationXSpeed(), path.getTranslationYSpeed(), path.getRotationSpeed());

        return chassisSpeeds;

    }
}
}
