// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team7111.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team7111.robot.subsystems.Aimbot;
import team7111.robot.subsystems.Autonomous;
import team7111.robot.subsystems.Hopper;
import team7111.robot.subsystems.Intake;
import team7111.robot.subsystems.Shooter;
import team7111.robot.subsystems.SuperStructure;
import team7111.robot.subsystems.Swerve;
import team7111.robot.subsystems.Vision;
import team7111.robot.subsystems.Field;
import team7111.robot.subsystems.SuperStructure.SuperState;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    private Field field;
    private Autonomous auto;
    private Swerve swerve = new Swerve();
    private Vision vision = new Vision();
    private Aimbot aimbot = new Aimbot(vision, swerve::getPose, swerve::getVelocity);
    private Intake intake = new Intake();
    private Hopper hopper = new Hopper();
    private Shooter shooter = new Shooter(aimbot);
    private SuperStructure superStructure;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        field = new Field(() -> {
            var alliance = DriverStation.getAlliance();
            if(alliance.isPresent()){
                return alliance.get() == Alliance.Red;
            }
            return false;
        });
        auto = new Autonomous(field);
        superStructure = new SuperStructure(auto, swerve, vision, aimbot, intake, hopper, shooter, field);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        long timerstart = System.nanoTime();
        CommandScheduler.getInstance().run();
        long timerend = System.nanoTime();

        SmartDashboard.putNumber("Total time of robot", (double) ((timerend-timerstart) / 1000000.0));
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        superStructure.setSuperState(SuperState.autonomousEnter);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void autonomousExit(){
        superStructure.disableAuto();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
