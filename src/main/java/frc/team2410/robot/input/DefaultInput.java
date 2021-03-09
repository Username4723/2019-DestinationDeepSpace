package frc.team2410.robot.input;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class DefaultInput extends InputManager {
    private final Joystick joystick;
    private final XboxController xbox;

    public DefaultInput() {
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
}
