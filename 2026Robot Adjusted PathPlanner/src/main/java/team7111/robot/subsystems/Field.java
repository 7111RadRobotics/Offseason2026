package team7111.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Field extends SubsystemBase {
    
    private BooleanSupplier isRedAlliance;

    public Field(BooleanSupplier isRedAlliance){
        this.isRedAlliance = isRedAlliance;
    }

    private boolean inAllianceZone(Pose2d pose, boolean isRedAlliance){
        if(isRedAlliance){
            return pose.getX() >= 11.9;
        }else{
            return pose.getX() <= 4.65;
        }
    }

    public boolean inAllianceZone(Pose2d pose){
        return inAllianceZone(pose, isRedAlliance.getAsBoolean());
    }

    public boolean inNeutralZone(Pose2d pose){
        return !inAllianceZone(pose, true) && !inAllianceZone(pose, false);
    }

    public Pose2d getHub(boolean isRedAlliance){
        return isRedAlliance
            ? new Pose2d( 4.635, 4.039, Rotation2d.kZero)
            : new Pose2d(11.946, 4.039, Rotation2d.kZero);
    }

    public Pose2d getAllianceHub(){
        return getHub(isRedAlliance.getAsBoolean());
    }
}
