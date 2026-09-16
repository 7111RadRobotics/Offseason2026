package team7111.robot.utils.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GenericGyro {

    public Rotation2d getYaw();

    public Rotation2d getPitch();

    public Rotation2d getRoll();

    public Rotation3d getRotation3d();

    public void setYaw(Rotation2d value);

    public void setPitch(Rotation2d value);

    public void setRoll(Rotation2d value);

    public void setRotation3d(Rotation3d value);

    /**
     * Sets the positive direction of the gyro
     * @param isInverted The inverted value with false being CCW+ and true being CW+
     */
    public void invertYaw(boolean isCCW);
    public void invertPitch(boolean isCCW);
    public void invertRoll(boolean isCCW);

    default public void zero(){
        setRotation3d(Rotation3d.kZero);
    }

    default public void update(){}
}
