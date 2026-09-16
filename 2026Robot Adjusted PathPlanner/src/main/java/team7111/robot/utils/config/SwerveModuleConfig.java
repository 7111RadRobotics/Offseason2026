package team7111.robot.utils.config;

import edu.wpi.first.math.geometry.Translation2d;
import team7111.robot.utils.encoder.GenericEncoder;

public class SwerveModuleConfig {
    public final SwerveMotorConfig driveMotor;
    public final SwerveMotorConfig angleMotor;
    public final GenericEncoder encoder;
    public final double canCoderOffsetDegrees;
    public final double wheelCircumference;
    public final Translation2d fromCenter;

    public SwerveModuleConfig(
        SwerveMotorConfig driveMotor, SwerveMotorConfig angleMotor, GenericEncoder encoder, 
        double canCoderOffsetDegrees, double wheelCircumference, Translation2d fromCenter
    ) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
        this.wheelCircumference = wheelCircumference;
        this.fromCenter = fromCenter;
    }
}
