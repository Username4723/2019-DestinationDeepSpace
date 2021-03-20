package frc.team2410.robot.config;

import lombok.Data;

@Data
public class WheelConfig {
    private final int steer;
    private final int steerEncoder;
    private final int drive;
    private final float offset;
    private final boolean inverted;
}
