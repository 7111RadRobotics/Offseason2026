package team7111.robot.utils.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team7111.robot.utils.config.SwerveModuleConfig;
import team7111.robot.utils.encoder.GenericEncoder;


public class CombinedSwerveModule implements GenericSwerveModule {

    private GenericSwerveModule driveMotor;
    private GenericSwerveModule angleMotor;
    
    private SwerveModuleConfig config;

    public CombinedSwerveModule(GenericSwerveModule driveMotor, GenericSwerveModule angleMotor, SwerveModuleConfig config) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;

        this.config = config;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        driveMotor.setOpenDriveState(state);
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        driveMotor.setClosedDriveState(state);
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getDriveVelocity();
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getDrivePosition();
    }

    @Override
    public Translation2d getOffset() {
        return config.fromCenter;
    }

    /** Gets the angle of the swerve drive */
    @Override
    public Rotation2d getAngle() {
        return angleMotor.getAngle();
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        angleMotor.setAngle(rotation);
    }

    @Override
    public GenericEncoder getEncoder() {
        return angleMotor.getEncoder();
    }

    @Override
    public void zeroWheels() {
        angleMotor.zeroWheels();
    }

    @Override
    public void configure() {
        angleMotor.configure();
        driveMotor.configure();
    }  
};