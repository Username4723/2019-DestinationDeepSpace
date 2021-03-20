package frc.team2410.robot.robots;

import frc.team2410.robot.config.RobotConfig;
import frc.team2410.robot.config.WheelConfig;
import frc.team2410.robot.config.WheelPosition;
import frc.team2410.robot.input.InputManager;

public class DefaultRobotConfig extends RobotConfig {
    public DefaultRobotConfig() {
        WHEELS.put(WheelPosition.FRONT_LEFT, new WheelConfig(7, 2, 8, 1.666259595f, true));
        WHEELS.put(WheelPosition.FRONT_RIGHT, new WheelConfig(5, 3, 6, 0.797119059f, false));
        WHEELS.put(WheelPosition.BACK_LEFT, new WheelConfig(1, 0, 2, 1.8701169960000001f, true));
        WHEELS.put(WheelPosition.BACK_RIGHT, new WheelConfig(3, 1, 4, 2.4316403760000003f, false));
    }

    @Override
    public InputManager createInputManager() {
        return new DefaultInputManager();
    }
}
