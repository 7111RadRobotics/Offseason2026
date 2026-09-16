package team7111.robot.utils.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class NavXGyro implements GenericGyro{
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private Rotation3d gyroValues = new Rotation3d();
    private Rotation2d yawOffset = Rotation2d.kZero;
    private Rotation2d pitchOffset = Rotation2d.kZero;
    private Rotation2d rollOffset = Rotation2d.kZero;

    private double yawInversion = 1;
    private double pitchInversion = 1;
    private double rollInversion = 1;

    private double permaOffset = 0;
    public NavXGyro(){
        
    }

    public Rotation2d getYaw(){
        return gyroValues.toRotation2d();
    }

    public Rotation2d getPitch(){
        return Rotation2d.fromRadians(gyroValues.getY());
    }

    public Rotation2d getRoll(){
        return Rotation2d.fromRadians(gyroValues.getX());
    }

    @Override
    public Rotation3d getRotation3d() {
        return gyroValues;
    }

    public void setYaw(Rotation2d rotation){
        gyro.reset();
        yawOffset = new Rotation2d(Units.degreesToRadians(rotation.getDegrees() + permaOffset));
    }

    public void setPitch(Rotation2d rotation){
        pitchOffset = rotation;
    }

    public void setRoll(Rotation2d rotation){
        rollOffset = rotation;
    }
    

    public void setRotation3d(Rotation3d rotation){
        yawOffset = rotation.toRotation2d();
        pitchOffset = Rotation2d.fromRadians(rotation.getY());
        rollOffset = Rotation2d.fromRadians(rotation.getX());
    }

    public void invertYaw(boolean isCCW){
        yawInversion = isCCW
            ? -1
            : 1;
    }

    public void invertPitch(boolean isCCW){
        pitchInversion = isCCW
            ? 1
            : -1;
    }

    public void invertRoll(boolean isCCW){
        rollInversion = isCCW
            ? 1
            : -1;
    }

    @Override
    public void update(){
        double yaw, pitch, roll;

        yaw = gyro.getYaw() + yawOffset.getDegrees();
        pitch = gyro.getPitch() + pitchOffset.getDegrees();
        roll = gyro.getRoll() + rollOffset.getDegrees();
        yaw = Units.degreesToRadians(limitNumbers(yaw));
        pitch = Units.degreesToRadians(limitNumbers(pitch));
        roll = Units.degreesToRadians(limitNumbers(roll));
        gyroValues = new Rotation3d(roll * rollInversion, pitch * pitchInversion, yaw * yawInversion);
    }

    private double limitNumbers(double unlimitedValue){
        if(unlimitedValue > 180){
            unlimitedValue = -180 + (unlimitedValue % 180);
        }else if(unlimitedValue < -180){
            unlimitedValue = 180 + (unlimitedValue % -180);
        }
        return unlimitedValue;
    }

}
