package team7111.robot.subsystems;

import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.Constants.MechanismConstants;

public class Aimbot extends SubsystemBase{
    /** Given as backup for if camera detects no valid apriltag */
    private Supplier<Pose2d> robotPose;
    private Supplier<Transform2d> robotVelocity;

    private StructPublisher<Pose2d> aimingPoint = 
                                    NetworkTableInstance.getDefault().getStructTopic("Aiming target", Pose2d.struct).publish();
    private StructPublisher<Pose2d> targetPoint = 
                                    NetworkTableInstance.getDefault().getStructTopic("Target", Pose2d.struct).publish();

    private Pose2d aimPoint = null;

    private final Pose3d blueHub = new Pose3d(4.635, 4.034536, Units.inchesToMeters(72), null); 
    private final Pose3d redHub = new Pose3d(11.946, 4.034536, Units.inchesToMeters(72), null); 

    //CONTROLLER
    /** Controls the manual firing, and adds angle if the stick is moved */
    private XboxController operatorController = null;
    /** Multiplied against the angle stick (left). Called 50x per second, so max rate is this number x50 */
    private double angleSensitivity = 0.005;
    /** Multiplied against the speed stick (Right). Called 50x per second, so max rate is this number x50 */
    private double speedSensitivity = 0.000001;

    /** On scale of 0 - 1, if the controller is less than this, ignores the value */
    private double controllerDeadzone = 0.1;

    /** How far the stick can override the angle in non manual shots (degrees) */
    private double angleOverrideRange = 10;
    /** How far the operator can override the speed in non manual shots (rpm) */
    private double speedOverrideRange = 1000;


    /** For apriltag detection and targetting */
    private Vision vision;

    /** Position of target to aim at. */
    private Pose3d targetPose;

    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetXOffset = 1.05918/2;
    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetHeightOffset = 2/3;

    /** Offset between rio yaw and direction the robot shoots */
    private final double rioToShooterOffset = -1.5;

    /** shooter wheel diameter, in meters */
    private final double shooterDiameter = Units.inchesToMeters(4);
    /** Auto calculated based on shooter diameter, in inches */
    private final double wheelCircumference = shooterDiameter * Math.PI;

    //ANGLE CONSTRAINTS
    /** Minimum shooter angle in degrees, from horizontal */
    private final double minShooterAngle = MechanismConstants.minHoodTraj;
    /** Maximum shooter angle in degrees, from horizontal */
    private final double maxShooterAngle = MechanismConstants.maxHoodTraj;
    
    private final double lowestShooterAngle = maxShooterAngle;
    //SPEED CONSTRAINTS
    /** Maximum rotations per minute allowable on the shooter (in RPM) */
    private final double maxShooterSpeed = 6000;
    /** Minimum rotations per minute allowable on the shooter (Overrided in off state, in RPM) */
    private final double minShooterSpeed = 0;

    //POSITION OFFSETS
    /** Offset from ground the ball leaves the shooter, in meters */
    private final double shooterHeightOffset = Units.inchesToMeters(20);
    /** Offset from the center of the robot to the shooter, in meters */
    private final double shooterXOffset = 0.0;
    /** Offset from the center of the robot to the shooter in meters */
    private double shooterYOffset = 0.15;
    /** Optimal rpm of the shooter wheel for max distance with continuous fire, in rotations per minute */
    private final double shooterOptimalSpeed = 1500;

    /** Extra multiplier to account for losses from drag, rpm loss from ball, ect */
    private final double RPMMult = 1.7;

    /** How far from horizontal the camera is, in degrees */
    private double cameraAngleOffset = 0.0;
    /** How far from horizontal the shooter is, in degrees */
    private double shooterAngleOffset = 0.0;

    /** Enables/disables the math calculations (saves calculation time)
     *  <p> If disabled, sets angle to minimum shooter angle and speed to 0 */
    private boolean isEnabled = true;

    /** Defines if we want to use vision */
    private boolean isUsingVision = false;

    /** Determines algorithm for aiming the shooter. <p>
     * Direct - Aims directly to the target, sets speed to max <p>
     * Parabolic - Aims to indirectly hit the target, arcing the ball <p>
     * Transport - Sets speed to 0, angle to the lowest possible <p>
     * Manual - Uses operator controls to aim and fire <p>
     * Apriltag - Targets directly to the most well seen apriltag, or continues current values if vision is disabled <p>
     * Preset - aims at the current preset shot type <p>
     * ShotTable - uses a shot table and interpolates where it should aim <p>
     * ShootOnTheMove - Uses leading to fire a ball rather than directly at the target. Also will shoot at corners if the robot is not in its alliance <p>
    */
    public enum shotType {
        Direct,
        Parabolic,
        Transport,
        Manual,
        Apriltag,
        Preset,
        ShotTable,
        ShootOnTheMove,
    }

    public enum presetShotType {
        Trench,
        RegHubShot,
        Tower,
        Default,
        Outpost,
    }

    /** each double is the angle per foot */
    private double[] shotTableAngles = {
        83.0, //1ft
        78.8, //2ft
        76.0, //3ft
        73.0, //4ft
        72.0, //5ft
        70.8, //6ft
        69.7, //7ft
        68.8, //8ft
        67.9, //9ft
        67.2, //10ft
        66.6, //11ft
        66.1, //12ft
        65.7, //13ft
        65.3, //14ft
        64.9, //15ft
    };

    /** each double is the speed per foot */
    private double[] shotTableSpeeds = {
        582.0, //1ft
        551.0, //2ft
        557.5, //3ft
        573.1, //4ft
        592.0, //5ft
        612.2, //6ft
        632.8, //7ft
        653.4, //8ft
        673.8, //9ft
        693.9, //10ft
        713.7, //11ft
        733.1, //12ft
        752.1, //13ft
        770.8, //14ft
        789.1, //15ft
        };

    /** max distance for the shot table in feet */
    private final int maxDist = 15; 
    /** min distance for the shot table in feet */
    private final int minDist = 1;

    // On the move code
    /** Time per iteration in moving shot code, in seconds */
    private final double timeStep = 0.05;
    /** Max time in seconds the robot will calculate, in seconds */
    private final double maxFltTime = 6;
    /** Starting calculation step for the shooter, in seconds */
    private final double startingTimeStep = 1.0;
    /** Minimum angle the ball is allowed to fall into the target while using on the move shooting, in degrees from horizontal (positive is downwards) */
    private final double minImpactAngle = 65;
    /** If no valid shooting solution is found, sets this to false. */
    private boolean possibleToFire = true;

    /** Used to see if the target has changed at all, set each time it goes through the loop */
    private Pose3d prevTarget = null;
    /** If the firing stays on on the move shooting, turns to true. */
    private boolean uninterruptedFiring = false;
    /** If has calculated the time to fire a ball already, and still using the same target and firing method (most likely havent moved much) */
    private double timeOffset = 0;
    /** Boolean to track if we are firing at the hub. when false, disables the on the move minimum impact angle check */
    private boolean hubShot = true;

    /** Current type of shot to calculate */
    private shotType currentShotType = shotType.Parabolic;

    /** Current preset to fire at if shot type is set to preset*/
    private presetShotType presetShot = presetShotType.Default;

    /** Calculated angle set in periodic method, in degrees (includes shooter and camera offsets in calculation already) */
    private double calculatedAngle = 82.0;
    /** Calculated speed the wheel needs to spin at, in rotations per minute */
    private double calculatedSpeed = 0.0; 
    /** The direction in degrees to the target */
    private double degreeToTarget = 0.0;

    public Aimbot(Vision vision, Supplier<Pose2d> robotPose, Supplier<Transform2d> robotVelocity) {
        this.vision = vision;
        this.robotPose = robotPose;
        this.robotVelocity = robotVelocity;
        
        //Defaults to shooting at the blue hub
        this.targetPose = blueHub;

        resetTarget();

        
    }

    /** Sets suppliers if not able to be given when aimbot class is initilized */
    public void giveResources(XboxController operatorController, BooleanSupplier isBlueAlliance) {
        this.operatorController = operatorController;
        if (isBlueAlliance != null) {
            if(isBlueAlliance.getAsBoolean()) {
                this.targetPose = blueHub;
            } else {
                this.targetPose = redHub;
            }
        }
    }

    /** Sets the angle offsets for the camera and the shooter, measured from horizontal */
    public void setOffsets(double cameraOffset, double shooterOffset) {
        this.cameraAngleOffset = cameraOffset;
        this.shooterAngleOffset = shooterOffset;
    }

    /** Returns if a shooting solution has been found. Only used with on the move shooting. */
    public boolean shotPossible() {
        return possibleToFire;
    }

    /** Returns calculated angle the shooter needs to fire at */
    public double getCalculatedAngle() {
        return this.calculatedAngle;
    }

    public double getCalculatedSpeed() {
        return this.calculatedSpeed;
    }

    public double getCalculatedDirection() {

        // Checks and ensures between -180 and 180
        double returnAngle = degreeToTarget + rioToShooterOffset;
        if(returnAngle < -180) {
            returnAngle += 360;
        } else if(returnAngle > 180) {
            returnAngle -= 360;
        }

        return returnAngle;
    }

    public boolean getToggle(){
        return isEnabled;
    }

    /** Enables or disables the autoshooting calculations */
    public void setToggle(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    /** Toggles the enable for calculations */
    public void toggle() {
        this.isEnabled = !isEnabled;
    }

    /** If set to true, vision will be used to find apriltag and target. If false, uses robot position */
    public void setVisionUsage(boolean isUsingVision) {
        this.isUsingVision = isUsingVision;
    }

    /** Toggles the vision usage */
    public void toggleVision() {
        isUsingVision = !isUsingVision;
    }

    /** Sets the current shot type to calculate */
    public void setShotType(shotType shotType) {
        currentShotType = shotType;
    }

    public shotType getShotType(){
        return currentShotType;
    }

    public void setPreset(presetShotType shotType) {
        //shotType = presetShot;
        presetShot = shotType;
    }

    public presetShotType getPresetShotType(){
        return presetShot;
    }

    /** Allows custom targeting to a target position. Does not affect anything if shooting with vision. */
    public void setCustomTarget(Pose3d customTarget) {
        targetPose = customTarget;
        hubShot = false;
    }
    
    /** Resets the target to default (the current alliance hub for most shooting modes, unless otherwise specified) */
    public void resetTarget() {
        if(DriverStation.isDSAttached()) {
            if(DriverStation.getAlliance().isPresent()) {
                if(DriverStation.getAlliance().get() == Alliance.Blue) {
                    targetPose = blueHub;
                } else {
                    targetPose = redHub;
                }
            } else {
                targetPose = blueHub;
            }
        }

        hubShot = true;
    }

    /** Calculates angle and speed for the shooter. If calculations are disabled, acts as a transport mode.*/
    public void periodic() {
        SmartDashboard.putBoolean("Is enabled", isEnabled);
        SmartDashboard.putBoolean("Is vision enabled", isUsingVision);
        SmartDashboard.putNumber("angle to target", degreeToTarget);
        SmartDashboard.putString("ShotType", currentShotType.toString());

        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
                shooterYOffset = -0.15;
            }
        }

        if(!isEnabled) {
            calculatedAngle = lowestShooterAngle;
            calculatedSpeed = 0;
            return;
        }
        
        // States to fire with
        switch (currentShotType) {
            case Direct:
                directShot();
                SmartDashboard.putBoolean("ShootDirect", true);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Parabolic:
                parabolicShot();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", true);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Transport:
                transport();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", true);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Manual:
                manual();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", true);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Apriltag:
                apriltag();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", true);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case ShootOnTheMove:
                shootOnTheMove();
                SmartDashboard.putBoolean("On the move", true);
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                break;
            case Preset:
                presetShot();
                break;
            case ShotTable:
                shotTable();
                break;
        }

        if(currentShotType != shotType.ShootOnTheMove) {
            uninterruptedFiring = false;
        }
        prevTarget = targetPose;

        useOffsets();
        useRestraints();

        double theta = Units.degreesToRadians(calculatedAngle);

        double mps = calculatedSpeed * wheelCircumference / 60.0 / RPMMult;
        double vy = mps * Math.sin(theta);
        double vx = mps * Math.cos(theta);

        SmartDashboard.putNumber("mps", mps);

        double launchHeight = shooterHeightOffset;
        double targetHeight = targetPose.getZ();

        // becomes nan
        double discriminant = vy * vy - 2.0 * 9.81 * (targetHeight - launchHeight);

        double sqrtD = Math.sqrt(discriminant);

        // two times: up-pass and down-pass
        double t1 = (vy - sqrtD) / 9.81;
        double t2 = (vy + sqrtD) / 9.81;

        // usually use larger positive time for descending hit
        double t = Math.max(t1, t2);

        double dist = vx * t;

        double wantedTime = (vy / 9.81) * 2;

        SmartDashboard.putNumber("Calculated Time", wantedTime);

        SmartDashboard.putNumber("Distance to calculated lead", dist);

        double xdif = Math.cos(Units.degreesToRadians(degreeToTarget)) * dist;
        double ydif = Math.sin(Units.degreesToRadians(degreeToTarget)) * dist;

        aimPoint = new Pose2d(robotPose.get().getX() + xdif, robotPose.get().getY() + ydif, new Rotation2d(0));

        SmartDashboard.putNumber("Time to impact", t);
        
        aimingPoint.set(aimPoint);

        Pose2d targetPose2d = new Pose2d(targetPose.getX(), targetPose.getY(), new Rotation2d(0));
        targetPoint.set(targetPose2d);

        SmartDashboard.putNumber("XVel", robotVelocity.get().getX());
        SmartDashboard.putNumber("YVel", robotVelocity.get().getY());


    }

    /** Aims using presets */
    private void presetShot() {

        switch (presetShot) {
            case Trench:
                calculatedSpeed = 2700 / RPMMult;
                calculatedAngle = 65;
                break;
            case RegHubShot:
                calculatedAngle = SmartDashboard.getNumber("Shooter angle entry", 79);
                calculatedSpeed = SmartDashboard.getNumber("Shooter speed entry", 2100 / RPMMult);
                break;
            case Tower:
                calculatedAngle = 71.75;
                calculatedSpeed = 3400 / RPMMult;
                break;
            case Default:
                calculatedSpeed = 0 / RPMMult;
                calculatedAngle = 0;
                break;
            case Outpost:
                calculatedSpeed = 1000 / RPMMult;
                calculatedAngle = 75;
                break;
        }
    }

    /** Interpolates closest based on a shot table and current distance to target */
    private void shotTable() {
            Transform3d distanceToTarget = getTransToTarget();

            double distance = distanceToTarget.getX() + shooterXOffset;
            distance = Units.metersToFeet(distance);
            if(distance > maxDist || distance < minDist) {
                return;
            }

            double angleDifference = shotTableAngles[(int) Math.ceil(distance) -1] - shotTableAngles[(int) distance -1];
            double speedDifference = shotTableSpeeds[(int) Math.ceil(distance) -1] - shotTableSpeeds[(int) distance -1];

            double interpMult = distance % 1;

            angleDifference = angleDifference * interpMult;
            speedDifference = speedDifference * interpMult;

            calculatedAngle = shotTableAngles[(int) distance] + angleDifference;
            calculatedSpeed = shotTableSpeeds[(int) distance] + speedDifference;
    }

    /** Aims directly at the target */
    private void directShot() {
        Transform3d distanceToTarget = getTransToTarget();
        
        if(distanceToTarget == null) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            return;
        }

        double height = distanceToTarget.getZ();
        double distance = distanceToTarget.getX() + shooterXOffset;

        //Pathagorean theorum
        double directDistance = height * height + distance * distance;
        directDistance = Math.sqrt(directDistance);

        calculatedAngle = Math.asin(height/directDistance);
        calculatedAngle = calculatedAngle * 180/Math.PI;

        calculatedSpeed = shooterOptimalSpeed;
    }

    /** Sets calculated angle and speed to arc a shot to the target. Uses a 3rd point that the ball will pass through to calculate.
     * <p> WARNING, Possibly heavy on processing */
    private void parabolicShot() {

        Transform3d CamToTarget = getTransToTarget();

        possibleToFire = true;
        
        if (CamToTarget == null) {
            possibleToFire = false;
            return;
        }

        final double g = 9.81;//9.81;

        // Horizontal distance from shooter release to target
        double distanceToTarget = CamToTarget.getX();

        // Vertical target height relative to shooter release
        double heightDifference = CamToTarget.getZ() + 0.15;

        //The distance to lip is half a meter from the target
        double distanceToLip = distanceToTarget - 0.5;
        //The height of the parabola must pass through 15 inches above the target, at distance to lip back
        double lipHeight = targetPose.getZ() + Units.inchesToMeters(20);

        // Convert lip height to shooter-relative coordinates
        double lipHeightRelative = lipHeight - shooterHeightOffset;

        // Basic geometry checks
        if (distanceToTarget <= 0.0 || distanceToLip <= 0.0 || distanceToLip >= distanceToTarget) {
            possibleToFire = false;
            return;
        }

        // Minimum launch angle required to clear the lip while still hitting the target
        double denom = distanceToLip - (distanceToLip * distanceToLip / distanceToTarget);
        if (denom <= 1e-9 || !Double.isFinite(denom)) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            possibleToFire = false;
            return;
        }

        double tanThetaClear =
            (lipHeightRelative - heightDifference * (distanceToLip * distanceToLip / (distanceToTarget * distanceToTarget))) / denom;

        if (!Double.isFinite(tanThetaClear)) {
            possibleToFire = false;
            return;
        }

        double thetaClear = Math.atan(tanThetaClear);

        // Minimum-speed angle to hit the target at all
        double thetaMinSpeed = Math.atan(
            (heightDifference + Math.sqrt(distanceToTarget * distanceToTarget
                                        + heightDifference * heightDifference))
            / distanceToTarget
        );

        // Choose the larger of:
        // - angle needed to clear lip (+ a little margin)
        // - minimum-speed angle
        double theta = Math.max(thetaClear + Units.degreesToRadians(3.0), thetaMinSpeed);

        // Clamp to shooter limits
        theta = Math.max(theta, Units.degreesToRadians(minShooterAngle));
        theta = Math.min(theta, Units.degreesToRadians(maxShooterAngle));

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        // Solve required launch speed to hit target at chosen angle
        double denomSpeed = 2.0 * cos * cos * (distanceToTarget * tan - heightDifference);

        if (denomSpeed <= 1e-9 || !Double.isFinite(denomSpeed)) {
            possibleToFire = false;
            return;
        }

        double speedSq = g * distanceToTarget * distanceToTarget / denomSpeed;

        if (speedSq <= 0.0 || !Double.isFinite(speedSq)) {
            possibleToFire = false;
            return;
        }

        double mps = Math.sqrt(speedSq);

        // Convert m/s to wheel RPM
        calculatedAngle = Units.radiansToDegrees(theta);
        calculatedSpeed = (mps / wheelCircumference) * 60.0;

        // Optional: if you still need your shooter multiplier model elsewhere,
        // then apply/remove RPMMult here consistently with the rest of your code.
    }

    /** Most heavy on processing, MONITER SPEED OF PROCESSOR */
    public void shootOnTheMove() {
        final double g = 9.81;
        final double paddingTime = 0.25;

        possibleToFire = false;

        Pose2d pose = robotPose.get();
        Transform2d vel = robotVelocity.get();

        double dx = targetPose.getX() - pose.getX();
        double dy = targetPose.getY() - pose.getY();
        double dz = targetPose.getZ() - shooterHeightOffset;

        // Convert robot-relative velocity to field-relative velocity
        double heading = pose.getRotation().getRadians();

        double robotVX =
            vel.getX() * Math.cos(heading) - vel.getY() * Math.sin(heading);

        double robotVY =
            vel.getX() * Math.sin(heading) + vel.getY() * Math.cos(heading);

        if (!(uninterruptedFiring && prevTarget == targetPose)) {
            timeOffset = 0;
        }

        boolean wasWithinConstraints = false;

        for (double t = startingTimeStep + timeOffset; t < maxFltTime; t += timeStep) {
            // Required launch velocity in FIELD frame
            double shotVX = dx / t - robotVX;
            double shotVY = dy / t - robotVY;
            double shotVZ = dz / t + 0.5 * g * t;

            double horizontalSpeed = Math.hypot(shotVX, shotVY);
            double shootingAngle = Units.radiansToDegrees(Math.atan2(shotVZ, horizontalSpeed));
            double shootingSpeedMps = Math.sqrt(shotVX * shotVX + shotVY * shotVY + shotVZ * shotVZ);
            double shootingSpeedRpm = (shootingSpeedMps / wheelCircumference) * 60.0;

            if (shootingSpeedRpm <= maxShooterSpeed &&
                shootingAngle >= minShooterAngle &&
                shootingAngle <= maxShooterAngle) {

                wasWithinConstraints = true;

                // Impact vertical velocity at target
                double impactVZ = shotVZ - g * t;

                // Negative means descending
                double impactAngleDeg = Units.radiansToDegrees(
                    Math.atan2(impactVZ, horizontalSpeed)
                );

                boolean impactOk = true;

                if (hubShot) {
                    impactOk = impactAngleDeg <= -minImpactAngle;
                }

                if (impactOk) {
                    calculatedAngle = shootingAngle;
                    calculatedSpeed = shootingSpeedRpm * 1.05;

                    // Final field-relative shot direction
                    degreeToTarget = Units.radiansToDegrees(Math.atan2(shotVY, shotVX));

                    timeOffset = t - startingTimeStep - paddingTime;
                    uninterruptedFiring = true;
                    possibleToFire = true;
                    return;
                }

            } else if (wasWithinConstraints) {
                uninterruptedFiring = false;
                possibleToFire = false;
                return;
            }
        }
        uninterruptedFiring = false;
        possibleToFire = false;
    }

    /** Sets angle to as close to horizontal as possible, and speed to 0 */
    private void transport() {
        calculatedAngle = lowestShooterAngle;
        calculatedSpeed = 2000 / RPMMult;
    }

    /** Sets to fire as flat of a line as possible. Operator controls do NOT determine raw angle, but distance they want to fire */
    private void manual() {
        possibleToFire = true;
        //Deadzone application
        if(Math.abs(operatorController.getLeftY()) > controllerDeadzone) {
            calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleSensitivity;
        }
        if(Math.abs(operatorController.getRightY()) > controllerDeadzone) {
            calculatedSpeed = operatorController.getRightY() * maxShooterSpeed;
        } else {
            calculatedSpeed = 0;
        }
    }

    /** Offsets the angle to use the proper angle reference */
    private void useOffsets() {
        if(isUsingVision) {
            calculatedAngle = calculatedAngle + cameraAngleOffset;
        }
        calculatedAngle = calculatedAngle + shooterAngleOffset;
        
        calculatedSpeed = calculatedSpeed *RPMMult;
        if(currentShotType != shotType.Manual)
        {
            //Deadzone application
            if(Math.abs(operatorController.getLeftY()) > controllerDeadzone) {
                calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleOverrideRange / 2;
            }
            if(Math.abs(operatorController.getRightY()) > controllerDeadzone) {
                calculatedSpeed = calculatedSpeed - operatorController.getRightY() * speedOverrideRange / 2;
            }
        }
    }

    //Shoots towards apriltag, or last variables if apriltags are null/disabled
    void apriltag() {
        if(!isUsingVision) {
            return;
        }

        if(getTransToTarget() == null) {
            return;
        }

        directShot();
    }
    
    /** Ensures the angles are within the min and max physical angles on the shooter */
    private void useRestraints() {
        SmartDashboard.putNumber("CalculatedAngle before clamped", calculatedAngle);
        SmartDashboard.putNumber("CalculatedSpeed before clamped", calculatedSpeed);

        if(calculatedAngle < minShooterAngle) {
            calculatedAngle = minShooterAngle;
        } else if(calculatedAngle > maxShooterAngle) {
            calculatedAngle = maxShooterAngle;
        }

        if(calculatedSpeed > maxShooterSpeed) {
            calculatedSpeed = maxShooterSpeed;
        } else if(calculatedSpeed < minShooterSpeed) {
            calculatedSpeed = minShooterSpeed;
        }
    }

    /** Gets a transform 3d object to the target, either using vision or absolute position to calculate <p>
     * X is distance to the target,
     * Z is height off ground,
     * Rotation is the direction you need to be pointing at the target, not an offset, in degrees.
     */
    private Transform3d getTransToTarget() {
        
        if(isUsingVision) {
            Transform3d visionResults = vision.cameraList[0].getCamToTarget();
            if( visionResults == null) {
                return null;
            }
            //Robot relative
            Transform3d calculatedPos = new Transform3d(
                visionResults.getX() - camToTargetXOffset - shooterXOffset, //Distance to the target from center of robot
                visionResults.getY() - shooterYOffset, //Left/Right offset of the robot to the target
                visionResults.getZ() + camToTargetHeightOffset, //Height of the target (from 0)
                null);


            //FOR ROTATION CALUCLATION
            //X difference and y difference
            double xdif = targetPose.getX() - robotPose.get().getX() - shooterXOffset;
            double ydif = targetPose.getY() - robotPose.get().getY() - shooterYOffset;

            double rotation = 0;
            
            rotation = Math.toDegrees(Math.atan2(ydif, xdif));

            degreeToTarget = rotation;

            rotation = 90 - rotation;

            //ROTATION CALCULATION

            double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
            Transform3d returnedTrans = new Transform3d(distance,
                0.0,
                calculatedPos.getZ(),
                new Rotation3d(new Rotation2d(rotation)));
            degreeToTarget = rotation;

            return returnedTrans;
        }
        //Field relative
        Transform3d calculatedPos = new Transform3d(targetPose.getX() - robotPose.get().getX(), //Left/Right distance to the target
            targetPose.getY() - robotPose.get().getY(), //Up/Down offset in 2d grid
            targetPose.getZ(), //Height of the target (from 0)
            null);
            
        //X difference and y difference
        double xdif = targetPose.getX() - robotPose.get().getX() - shooterXOffset;
        double ydif = targetPose.getY() - robotPose.get().getY() - shooterYOffset;

        double rotation = 0;
        
        rotation = Math.toDegrees(Math.atan2(ydif, xdif));

        double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
        Transform3d returnedTrans = new Transform3d(
            distance, 
            0.0, 
            targetPose.getZ() - shooterHeightOffset, 
            new Rotation3d(new Rotation2d(rotation)));
        degreeToTarget = rotation;

        SmartDashboard.putNumber("Distance", Units.metersToFeet(distance));
        return returnedTrans;
    }
}
//floccinaucinihilipilification
//hehe >:3