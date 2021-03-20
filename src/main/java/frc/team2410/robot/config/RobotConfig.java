package frc.team2410.robot.config;

import frc.team2410.robot.input.InputManager;

import java.util.HashMap;
import java.util.Map;

public abstract class RobotConfig {
    public final Map<WheelPosition, WheelConfig> WHEELS = new HashMap<>();

    public abstract InputManager createInputManager();
}
