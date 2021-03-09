package frc.team2410.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

@Deprecated
public class UserInput {
	private Robot robot;

	private final boolean[][] canPress = new boolean[2][12];
	private final GenericHID[] controllers = new GenericHID[2];
	private final Joystick joy;
	private final XboxController xbox;

	UserInput(Robot robot) {
		this.robot = robot;

		joy = new Joystick(0);
		xbox = new XboxController(1);
		controllers[0] = joy;
		controllers[1] = xbox;
	}

	public double getX() {
		return this.applyDeadzone(joy.getRawAxis(0), 0.05, 1);
	}

	public double getY() {
		return this.applyDeadzone(-joy.getRawAxis(1), 0.05, 1);
	}

	public double getTwist() {
		return this.applyDeadzone(joy.getRawAxis(2), 0.01, 1) / 2;
	}

	public double getSlider() {
		return (1 - joy.getRawAxis(3)) / 2;
	}

	public double getAnalogStick(boolean rightStick, boolean yAxis) {
		return this.applyDeadzone(xbox.getRawAxis((rightStick ? 1 : 0) * 2 + (yAxis ? 1 : 0)), 0.25, 1);
	}

	public boolean startPressed() {
		return xbox.getRawButton(10);
	}

	public double getJoyPOV() {
		return this.joy.getPOV(0);
	}

	private double applyDeadzone(double val, double deadzone, double maxval) {
		if (Math.abs(val) <= deadzone) return 0;
		double sign = val / Math.abs(val);
		val = sign * maxval * (Math.abs(val) - deadzone) / (maxval - deadzone);
		return val;
	}
}
