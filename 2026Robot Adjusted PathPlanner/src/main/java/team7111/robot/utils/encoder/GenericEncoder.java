package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GenericEncoder {
    
    public Rotation2d getPosition();

    public void setPosition(Rotation2d rotation);
    
    default public void periodic(){}
}
