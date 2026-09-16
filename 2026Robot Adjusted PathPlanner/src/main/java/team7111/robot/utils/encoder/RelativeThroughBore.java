package team7111.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;

public class RelativeThroughBore implements GenericEncoder {

    private Encoder encoder;
    private double offset = 0;

    private double conversionFactor = 1;

    public RelativeThroughBore(int channelA, int channelB, boolean isCCW, double conversionFactor, double offset) {
        encoder = new Encoder(channelA, channelB);
        encoder.setDistancePerPulse(conversionFactor);
        encoder.setReverseDirection(!isCCW);
        this.conversionFactor = conversionFactor;
        this.offset = offset;
    }

    @Override
    public Rotation2d getPosition() {
       return Rotation2d.fromDegrees((((double)encoder.get() / 2048.0) * 1/conversionFactor) * 360 + offset);
    }

    @Override
    public void setPosition(Rotation2d desired) {
        encoder.reset();
    }
}
