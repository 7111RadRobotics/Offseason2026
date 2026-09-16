package team7111.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.utils.swervemodules.GenericSwerveModule;

public class SwerveModule {
    public final int moduleNumber;
    public final GenericSwerveModule module;

    public SwerveModule(int moduleNumber, GenericSwerveModule moduleType) {
        this.moduleNumber = moduleNumber;
        module = moduleType;

        module.configure();
        module.zeroWheels();
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        // System.out.println("Angle: " + state.angle.getRadians() + "Mod #: " + moduleNumber);

        SmartDashboard.putNumber("wanted drive velocity", state.speedMetersPerSecond);
        state.optimize(getAngle());

        if (isOpenLoop) {
            module.setOpenDriveState(state);
        } else {
            module.setClosedDriveState(state);
        }
        module.setAngle(state.angle);
    }

    public SwerveModuleState getState() {
        SmartDashboard.putNumber("returned drive velocity", module.getDriveVelocity());
        return new SwerveModuleState(module.getDriveVelocity(), getAngle());
    }

    public Rotation2d getEncoder() {
        return module.getEncoder().getPosition();
    }

    public Rotation2d getAngle() {
        Rotation2d rot = module.getAngle();
        return rot;
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition pos = new SwerveModulePosition(module.getDrivePosition(), getAngle());

        return pos;
    }

    public Translation2d getOffset() {
        return module.getOffset();
    }
    /** Returns in rotations per minute */
    public double getVelocity() {
        return module.getDriveVelocity() * 60;
    }
}
