package team7111.robot.utils.gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class SimSwerveGyro extends SimGyro {

    private SwerveModuleState[] states;
    private Supplier<SwerveModuleState[]> statesSupplier;
    private SwerveDriveKinematics kinematics;
    private AnalogGyro gyro = new AnalogGyro(0);
    private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
    private Rotation2d gyroRotation = Rotation2d.kZero;

    private double updateRate = 0.02;
    private double invertValue = 1;

    public SimSwerveGyro(Supplier<SwerveModuleState[]> states, SwerveDriveKinematics kinematics){
        this.states = states.get();
        statesSupplier = states;
        this.kinematics = kinematics;

        gyroSim.setAngle(0);
    }

    @Override
    public Rotation2d getYaw() {
        return gyroRotation;
    }

    @Override
    public void setYaw(Rotation2d rotation) {
        gyroSim.setAngle(rotation.getDegrees());
    }

    @Override
    public void invertYaw(boolean isCCW) {
        invertValue = isCCW
            ? -1
            : 1;
    }

    @Override
    public void update(){
        states = statesSupplier.get();

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
        double degreeRate = Units.radiansToDegrees(speeds.omegaRadiansPerSecond *invertValue) * updateRate;
        double rotationDegrees = gyroSim.getAngle();

        rotationDegrees += degreeRate;
        if(rotationDegrees > 180){
            rotationDegrees = -180 + (rotationDegrees % 180);
        }else if(rotationDegrees < -180){
            rotationDegrees = 180 - (rotationDegrees % -180);
        }

        gyroRotation = Rotation2d.fromDegrees(rotationDegrees);
        gyroSim.setAngle(rotationDegrees);
    }

}
