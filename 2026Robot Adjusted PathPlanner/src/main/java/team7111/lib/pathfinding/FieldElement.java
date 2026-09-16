package team7111.lib.pathfinding;


import edu.wpi.first.math.geometry.Pose2d;


public class FieldElement {
    

    //For circle
    private Pose2d center;
    private double radius;

    //For polygon
    private Pose2d corners[];

    /**
     * Constructs field element using a pose2d for each corner. Creates a polygon object
     * @param corners -Array of pose2d. 1 for each corner. Link sequentially. (corner 1 -> corner 2 -> corner 3 -> corner 4 -> corner 1)
     */
    public FieldElement(Pose2d corners[])
    {
        this.corners = corners;
    }

    /**
     * Constructs field element using a pose2d for the center point. Creates a circle object.
     * @param center -Pose2d. Center of the circle
     * @param radius -Double in meters. Distance edge is to center.
     */
    public FieldElement(Pose2d center, double radius)
    {
        this.center = center;
        this.radius = radius;
    }


    /**
     * Returns array of pose2d. Each element is a corner in sequential order. Returns null if not a polygon.
     */
    public Pose2d[] returnCorners()
    {
        if(corners == null)
        {
            return null;
        }
        return corners;
    }

    /**
     * Returns 2 pose2ds. 1st one is the center, 2nd one is a point on the outside ring edge. Returns null if the object is not a circle
     * <p>
     * Take difference from points in y to find radius.
     * 
     */
    public Pose2d[] returnCirclePose()
    {
        //Checks if null (not a circle)
        if(center == null)
        {
            return null;
        }

        Pose2d returnPose2d[] = new Pose2d[] 
        {
            center, new Pose2d(center.getX(), center.getY() + radius , null)
        };
        return returnPose2d;
    }
}
