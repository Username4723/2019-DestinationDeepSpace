package frc.team2410.robot.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class InputManager {
    private Map<InputSource, List<Boolean>> hasBeenPressed = new HashMap<>();
    public abstract boolean getButtonState(InputSource source, int buttonIndex);
    public abstract int getPOV(InputSource source);
    public abstract double getX();
    public abstract double getY();
    public abstract double getTwist();
    public abstract double getSlider();


    public final boolean getLeadingButtonState(InputSource source, int buttonIndex) {
        if (hasBeenPressed.containsKey(source) && hasBeenPressed.get(source).size() > buttonIndex && hasBeenPressed.get(source).get(buttonIndex)) return false;
        boolean currentState = getButtonState(source, buttonIndex);

        if (currentState) hasBeenPressed.computeIfAbsent(source, it -> new ArrayList<>()).set(buttonIndex, true);
        return currentState;
    }
}
