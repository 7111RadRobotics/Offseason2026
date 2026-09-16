package team7111.lib.pathfinding;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Path {

    private double mapLengthX = 16.5354; // width of field in meters
    private double mapLengthY = 8.001; // length of field in meters

    private boolean isflipped = false;

    //Robot position as a supplied Pose2d
    private Supplier<Pose2d> robotPose;

    //Translation speed of the robot in each axis, and rotation.
    private DoubleSupplier xTransSpeed = () -> 0.0;
    private DoubleSupplier yTransSpeed = () -> 0.0;
    private DoubleSupplier rotSpeed = () -> 0.0;

    public boolean firstSpeed = false;

    private double lastXSpeed = 0;
    private double lastYSpeed = 0;
    private double lastRotSpeed = 0;


    private int currentWaypointIndex = 0;
    
    private Waypoint[] waypoints;
    private boolean isPathFinished = false;

    /**
     * Constructs a path from several waypoints. Uses pathMaster class to define parameters.
     * @param waypoints -An array of waypoint objects.
     * @param robotPose -A supplier to be assigned to the local variable, letting path know robot pose.
     */
    public Path(Waypoint[] waypoints){
        this.waypoints = waypoints;

        var field = new Field2d();
        int index = 0;
        for(Waypoint waypoint : waypoints){
            field.getObject("Waypoint " + index).setPose(waypoint.getPose());
            index ++;
        }
        SmartDashboard.putData("Path Waypoints", field);
    }

    public Path(List<Waypoint> waypointList){
        this(waypointList.toArray(new Waypoint[waypointList.size()]));
    }

    /**
     * Returns the current waypoint the robot is pathing to.
     * <p>Note current waypoint index is one ahead of array for waypoint.
     */
    public int getCurrentWaypointIndex(){
        return currentWaypointIndex;
    }

    /**
     * Returns the current waypoint object that the robot is pathing to.
     */
    public Waypoint getCurrentWaypoint(){
        return waypoints[currentWaypointIndex];
    }

    public Waypoint[] getWaypoints(){
        return waypoints;
    }
    
    /**
     * Returns in degrees per second, positive clockwise.
     */
    public double getRotationSpeed(){
        double speed = rotSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getRotationConstraints();
        if(speed > waypointConstraints.getMaxSpeed()) {
            speed = waypointConstraints.getMaxSpeed();
        }
        if(speed < -waypointConstraints.getMaxSpeed()){
            speed = -waypointConstraints.getMaxSpeed();
        }
        if(Math.abs(speed) < waypointConstraints.getMinSpeed()){
            if(speed > 0){
                speed = waypointConstraints.getMinSpeed();
            }else{
                speed = -waypointConstraints.getMinSpeed();
            }
        }
        //speed = rotSpeed.getAsDouble();
        return speed;
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationXSpeed(){
        double combinedSpeed = Math.hypot(xTransSpeed.getAsDouble(), yTransSpeed.getAsDouble());
        double angle = Math.atan2(yTransSpeed.getAsDouble(), xTransSpeed.getAsDouble());
        double speed = xTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();

        if(Math.abs(combinedSpeed) > waypointConstraints.getMaxSpeed()){
            if(combinedSpeed > waypointConstraints.getMaxSpeed()){
                speed = Math.cos(angle) * waypointConstraints.getMaxSpeed();
            }else {
                speed = Math.cos(angle) * -waypointConstraints.getMaxSpeed();
            }
        }
        
        if(Math.abs(combinedSpeed) < waypointConstraints.getMinSpeed()){
            if(combinedSpeed > 0){
                speed = Math.cos(angle) * waypointConstraints.getMinSpeed();
            }else{
                speed = Math.cos(angle) * -waypointConstraints.getMinSpeed();
            }
        }

        double acceleration = Math.abs(speed) - Math.abs(lastXSpeed);

        if (acceleration > 0.06) {
            if (speed > 0 && speed > lastXSpeed) {
                speed = lastXSpeed + 0.06;
            } else if (speed < 0 && speed < lastXSpeed) {
                speed = lastXSpeed - 0.06;
            }
        } else if (acceleration < -0.06) {
            if (speed > 0 && speed < lastXSpeed) {
                speed = lastXSpeed - 0.06;
            } else if (speed <= 0 && speed > lastXSpeed) {
                speed = lastXSpeed + 0.06;
            }
        } 
        SmartDashboard.putNumber(("xSpeed"), speed);

        if (firstSpeed) {
            return lastXSpeed;
        } else {
            return speed;
        }

      
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationYSpeed(){
        double combinedSpeed = Math.hypot(xTransSpeed.getAsDouble(), yTransSpeed.getAsDouble());
        double angle = Math.atan2(yTransSpeed.getAsDouble(), xTransSpeed.getAsDouble());
        double speed = yTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();
        
        if(Math.abs(combinedSpeed) > waypointConstraints.getMaxSpeed()){
            if(combinedSpeed > waypointConstraints.getMaxSpeed()){
                speed = Math.sin(angle) * waypointConstraints.getMaxSpeed();
            }else {
                speed = Math.sin(angle) * -waypointConstraints.getMaxSpeed();
            }
        }
        
        if(Math.abs(combinedSpeed) < waypointConstraints.getMinSpeed()){
            if(combinedSpeed > 0){
                speed = Math.sin(angle) * waypointConstraints.getMinSpeed();
            }else{
                speed = Math.sin(angle) * -waypointConstraints.getMinSpeed();
            }
        }

        double acceleration = Math.abs(speed) - Math.abs(lastYSpeed);

        if (acceleration > 0.06) {
            if (speed > 0 && speed > lastYSpeed) {
                speed = lastYSpeed + 0.06;
            } else if (speed < 0 && speed < lastYSpeed) {
                speed = lastYSpeed - 0.06;
            }
        } else if (acceleration < -0.06) {
            if (speed > 0 && speed < lastYSpeed) {
                speed = lastYSpeed - 0.06;
            } else if (speed <= 0 && speed > lastYSpeed) {
                speed = lastYSpeed + 0.06;
            }
        } 

        SmartDashboard.putNumber("ySpeed", speed);

        if (firstSpeed) {
            return lastYSpeed;
        }
        else {
            return speed;
        }



        
        
    }

    public ChassisSpeeds getChassisSpeeds(){
        double dt = 1.0/50.0;

        double desCompSpeed = Math.hypot(xTransSpeed.getAsDouble(), yTransSpeed.getAsDouble());
        double xSpeed = xTransSpeed.getAsDouble();
        double ySpeed = yTransSpeed.getAsDouble();
        WaypointConstraints constraints = getCurrentWaypoint().getTranslationConstraints();

        if (desCompSpeed > constraints.getMaxSpeed()) {
            xSpeed = xSpeed * constraints.getMaxSpeed() / desCompSpeed;
            ySpeed = ySpeed * constraints.getMaxSpeed() / desCompSpeed;
            }  
        
        if (desCompSpeed < constraints.getMinSpeed()) {
            xSpeed = xSpeed * constraints.getMinSpeed() / desCompSpeed;
            ySpeed = ySpeed * constraints.getMinSpeed() / desCompSpeed;
        }

        double dx = xSpeed - lastXSpeed;
        double dy = ySpeed - lastYSpeed;

        double deltaV = Math.hypot(dx, dy);

        if (deltaV > 0.06) {
            double scale =  constraints.getMaxAccel() *dt / deltaV;
            dx *= scale;
            dy *= scale;

            xSpeed = lastXSpeed + dx;
            ySpeed = lastYSpeed + dy;
        }

        ChassisSpeeds limitSpeed = new ChassisSpeeds(xSpeed, ySpeed, getRotationSpeed());

        setLastSpeeds(limitSpeed.vxMetersPerSecond, limitSpeed.vyMetersPerSecond, getRotationSpeed());
        
        return limitSpeed;
        
    }

    

    /**
     * Sets the suppliers for speed on the robot to be equal to local variables.
     * @param xSpeed -X axis speed in meters per second.
     * @param ySpeed -Y axis speed in meters per second.
     * @param rotSpeed -Rotation speed in digrees per second.
     */
    public void setSpeedSuppliers(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
        xTransSpeed = xSpeed;
        yTransSpeed = ySpeed;
        this.rotSpeed = rotSpeed;
    }

    public void setLastSpeeds(double xSpeed, double ySpeed, double rotSpeed) {
        lastXSpeed = xSpeed;
        lastYSpeed = ySpeed;
        lastRotSpeed = rotSpeed;
    }

    public ChassisSpeeds getSpeeds() {
        return new ChassisSpeeds(lastXSpeed, lastYSpeed, lastRotSpeed);
    }

    /**
     * Sets the path robot position equal to an outside pose supplier.
     */
    public void setPoseSupplier(Supplier<Pose2d> pose){
        robotPose = pose;
    }
    
    /**
     * Returns if the path is finished or not
     */
    public boolean isPathFinished(){
        return isPathFinished;
    }

    /**
     * Resets key variables
     */
    public void initialize(){
        currentWaypointIndex = 0;
        isPathFinished = false;
    }

    /**
     * Flips the waypoint positions and rotations of all waypoints
     */
    public void flipPath(boolean isRedAlliance, boolean isMirrored){
        
        //X and y from origins.
        double length = mapLengthX;
        double width = mapLengthY;
        if(isflipped != isRedAlliance){
            for(int i = 0; i < waypoints.length; i++) {
                //Gets waypoint position
                double waypointX = waypoints[i].getPose().getX();
                double waypointY = waypoints[i].getPose().getY();
                double waypointRot = waypoints[i].getPose().getRotation().getDegrees();

                double newWayX;
                double newWayY;
                double newWayRot;
                
                //flips
                newWayX = -waypointX + length;
                newWayY = waypointY;
            
                //Rotation flipping
                
                newWayRot = (waypointRot + 180) * -1;
                newWayRot = newWayRot % 180; //Sets angle to within 360.

                if(!isMirrored){
                    newWayY = -waypointY + width;
                    if(waypointRot > 0){
                        newWayRot = waypointRot - 180;
                    }else{
                        newWayRot = waypointRot + 180;
                    }

                    if(!isRedAlliance){
                        newWayRot -= 180;
                    }
                }

                Waypoint newWaypoint = new Waypoint(new Pose2d(newWayX, newWayY, Rotation2d.fromDegrees(newWayRot)), 
                    waypoints[i].getTranslationConstraints(), waypoints[i].getRotationConstraints());

                waypoints[i] = newWaypoint;
                System.out.println("waypoint flipped");
                
            }
            isflipped = isRedAlliance;
        }
    }

    /**
     * Routs around any objects in the path
     */
    public void avoidFieldElements(boolean avoidFieldElements, FieldElement[] fieldElements){
        if(!avoidFieldElements){
            return;
        }

        
    }
    
    /**
     * indexes waypoint to path to if there. 
     * If path is finished, sets path to finished and will not path to new waypoint.
     */
    public void periodic(){
        SmartDashboard.putBoolean("firstSpeed?", firstSpeed);
        firstSpeed = false;

        SmartDashboard.putNumber("Last X Speed", lastXSpeed);
        SmartDashboard.putNumber("Last Y Speed", lastYSpeed);
        SmartDashboard.putNumber("Last Rot Speed", lastRotSpeed);
        if(robotPose == null){
            System.out.println("RobotPose null");
            return;
        }
        if(waypoints == null){
            System.out.println("waypoints null");
            return;
        }
        if(waypoints[currentWaypointIndex].isAtWaypoint(robotPose.get(), new ChassisSpeeds(getTranslationXSpeed(), getTranslationYSpeed(), getRotationSpeed())))
        {
            System.out.println("Next Waypoint");
            if(currentWaypointIndex == waypoints.length - 1){
                isPathFinished = true;
                System.out.println("Path Finished");
                return;
            }
            currentWaypointIndex++;
            firstSpeed = true;
        }
    }

    /**
     * Decorates every Waypoint of the Path with new translation constraints
     * @param constraints -the new translation constraints to use
     * @return itself for method chaining
     */
    public Path withTranslationConstraints(WaypointConstraints constraints){
        for (Waypoint waypoint : waypoints) {
            waypoint.withTranslationConstraints(constraints);
        }
        return this;
    }

    /**
     * Decorates every Waypoint of the Path with new rotation constraints
     * @param constraints -the new rotation constraints to use
     * @return itself for method chaining
     */
    public Path withRotationConstraints(WaypointConstraints constraints){
        for (Waypoint waypoint : waypoints) {
            waypoint.withRotationConstraints(constraints);
        }
        return this;
    }

    public Path withRotation(double degrees){
        for (Waypoint waypoint : waypoints) {
            waypoint.withRotation(degrees);
        }
        return this;
    }
}
