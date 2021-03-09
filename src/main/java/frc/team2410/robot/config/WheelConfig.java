package frc.team2410.robot.config;

public class WheelConfig {
    public final int STEER;
    public final int STEER_ENCODER;
    public final int DRIVE;
    public final float OFFSET;
    public final boolean INVERTED;

    public WheelConfig(int STEER, int STEER_ENCODER, int DRIVE, float OFFSET, boolean INVERTED) {
        this.STEER = STEER;
        this.STEER_ENCODER = STEER_ENCODER;
        this.DRIVE = DRIVE;
        this.OFFSET = OFFSET;
        this.INVERTED = INVERTED;
    }
}
