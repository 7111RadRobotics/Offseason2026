package team7111.robot.utils.encoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team7111.robot.Constants;

public class CTREEncoder implements GenericEncoder{

    private CANcoder encoder;
    private double offset;

    public CTREEncoder(int id, CANcoderConfiguration config){
        encoder = new CANcoder(id, Constants.canbus);
        encoder.getConfigurator().apply(config);
    }

    public CTREEncoder(int id, CANcoderConfiguration config, double offset){
        encoder = new CANcoder(id, Constants.canbus);
        encoder.getConfigurator().apply(config);
        this.offset = offset;
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble() + Units.degreesToRotations(offset));
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        encoder.setPosition(rotation.getRotations());
    }
}
