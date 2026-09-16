package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimDIOEncoder implements GenericEncoder {

    private EncoderSim encoderSim;
    private edu.wpi.first.wpilibj.Encoder encoder;

    private SimDIOEncoder(int channel){
        encoder = new edu.wpi.first.wpilibj.Encoder(channel, channel + 1);
        encoderSim = new EncoderSim(encoder);
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(encoder.getDistance());
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        encoderSim.setDistance(rotation.getRotations());
    }

}
