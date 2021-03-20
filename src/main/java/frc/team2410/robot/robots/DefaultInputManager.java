package frc.team2410.robot.robots;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.team2410.robot.input.InputManager;
import frc.team2410.robot.input.InputSource;

public class DefaultInputManager extends InputManager {
    private final Joystick joystick;
    private final XboxController xbox;

    public DefaultInputManager() {
        joystick = new Joystick(0);
        xbox = new XboxController(1);
    }

    @Override
    public boolean getButtonState(InputSource source, int buttonIndex) {
        switch (source) {
            case XBOX:
                return xbox.getRawButton(buttonIndex);
            case JOYSTICK:
                return joystick.getRawButton(buttonIndex);
        }

        throw new UnsupportedOperationException();
    }

    @Override
    public int getPOV(InputSource source) {
        switch (source) {
            case XBOX:
                return xbox.getPOV();
            case JOYSTICK:
                return joystick.getPOV();
        }

        throw new UnsupportedOperationException();
    }

    public double getX() {
        return this.applyDeadzone(joystick.getRawAxis(0), 0.05);
    }

    public double getY() {
        return this.applyDeadzone(-joystick.getRawAxis(1), 0.05);
    }

    public double getTwist() {
        return this.applyDeadzone(joystick.getRawAxis(2), 0.01) / 2;
    }

    public double getSlider() {
        return (1 - joystick.getRawAxis(3)) / 2;
    }

    public double getAnalogStick(boolean rightStick, boolean yAxis) {
        return this.applyDeadzone(xbox.getRawAxis((rightStick ? 1 : 0) * 2 + (yAxis ? 1 : 0)), 0.25);
    }

    private double applyDeadzone(double val, double deadzone) {
        if (Math.abs(val) <= deadzone) return 0;
        double sign = val / Math.abs(val);
        return sign * (Math.abs(val) - deadzone) / (1 - deadzone);
    }
}
