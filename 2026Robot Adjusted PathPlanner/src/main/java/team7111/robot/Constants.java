package team7111.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import team7111.robot.utils.config.DrivebaseConfig;
import team7111.robot.utils.config.SwerveModuleConfig;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
    public static String canbus = "rio";

    /** All joystick, button, and axis IDs. */
    public static class ControllerConstants {

        // Prevent from acclerating/decclerating to quick
        public static final SlewRateLimiter xDriveLimiter = new SlewRateLimiter(4);
        public static final SlewRateLimiter yDriveLimiter = new SlewRateLimiter(4);
        public static final SlewRateLimiter rotationLimiter = new SlewRateLimiter(4);
    }

    /** Contains all constants relate to mechanisms */

    public static class MechanismConstants {
        
        public static final double maxHoodPos = 32;
        public static final double minHoodPos = 8;
        public static final double maxHoodTraj = 90.0 - minHoodPos; // 82 //83
        public static final double minHoodTraj = 90.0 - maxHoodPos; // 58 60 //59.038

        public static final int shooterID = 16;
        public static final int shooterFollowerID = 17;
        public static final int shooterHoodID = 18;

        public static final int spindexerID = 14;
        public static final int shooterIndexerID = 15;
        
        public static final int intakePivotID = 12;
        public static final int intakeWheelsID = 10;
        public static final int intakeWheelsFollowerID = 11;
    }

    /** All swerve constants. */
    public static class SwerveConstants {
        /** Contains robot-specific drivebase constants */
        public static final DrivebaseConfig drivebaseConfig = DrivebaseConfig.getWhiplash(RobotBase.isSimulation());
        
        /** Constants that apply to the whole drive train. */
        public static final double wheelBaseWidth = drivebaseConfig.width; // Width of the drivetrain measured from the middle of the wheels.
        public static final double wheelBaseLength = drivebaseConfig.length; // Length of the drivetrain measured from the middle of the wheels.
        public static final double wheelDiameter = drivebaseConfig.wheelDiameter;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBaseLength / 2.0, wheelBaseWidth / 2.0),
            new Translation2d(wheelBaseLength / 2.0, -wheelBaseWidth / 2.0),
            new Translation2d(-wheelBaseLength / 2.0, wheelBaseWidth / 2.0),
            new Translation2d(-wheelBaseLength / 2.0, -wheelBaseWidth / 2.0)
        );

        public static final double driveGearRatio = drivebaseConfig.moduleConstants[0].driveMotor.gearRatio;
        public static final double driveRotationsToMeters = wheelCircumference / driveGearRatio;
        public static final double driveRPMToMPS = driveRotationsToMeters / 60.0;
        public static final double angleGearRatio = drivebaseConfig.moduleConstants[0].angleMotor.gearRatio;
        public static final double angleRPMToRPS = angleGearRatio / 60;

        /** Speed ramp. */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        
        /** Swerve constraints. */
        public static final double maxDriveVelocity = 7.5;
        public static final double maxAngularVelocity = 9;
        public static final double sensitivity = 1;

        /** 
d         * Module specific constants.c
         * CanCoder offset is in DEGREES, not radians like the rest of the repo.
         * This is to make offset slightly more accurate and easier to measure.
         */
        public static final SwerveModuleConfig mod0Constants = drivebaseConfig.moduleConstants[0];

        public static final SwerveModuleConfig mod1Constants = drivebaseConfig.moduleConstants[1];

        public static final SwerveModuleConfig mod2Constants = drivebaseConfig.moduleConstants[2];

        public static final SwerveModuleConfig mod3Constants = drivebaseConfig.moduleConstants[3];

        /** Motor direction */
        public static final boolean driveInversion = mod0Constants.driveMotor.Inverted;
        public static final boolean angleInversion = mod0Constants.angleMotor.Inverted;

        /** Idle modes */
        public static final boolean driveBreakMode = mod0Constants.driveMotor.isBrakeMode;
        public static final boolean angleBreakMode = mod0Constants.angleMotor.isBrakeMode;

        /** PID Controllers */
        public static final PIDController drivePID = mod0Constants.driveMotor.pid;
        public static final PIDController anglePID = mod0Constants.angleMotor.pid;

        /** Current limiting. */
        public static final int driveCurrentLimit = 40;
        public static final int angleCurrentLimit = 40;
    }

    public static class kAuto {
        /** PID Values. */
        
        /** Constraints. */
        public static final double maxDriveVelocity = 2.0;
        public static final double maxAngleVelocity = 5.0;
    }
}
