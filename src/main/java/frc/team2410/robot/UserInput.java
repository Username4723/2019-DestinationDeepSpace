package frc.team2410.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import static frc.team2410.robot.RobotMap.*;

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

	void pollButtons() {
		if (joy.getRawButton(5)) {
			robot.drivetrain.returnWheelsToZero();
		}

		if (joy.getRawButton(6)) {
			robot.drivetrain.resetHeading(0);
		} else if (joy.getRawButton(7)) {
			robot.drivetrain.resetHeading(180);
		}

		boolean resetPlace = true;

		if (joy.getRawButton(11)) {
			robot.semiAuto.turnToNearestAngle(180);
			resetPlace = false;
		} else if (joy.getRawButton(9)) {
			robot.semiAuto.turnToNearestAngle(0);
			resetPlace = false;
		} else if (joy.getRawButton(12)) {
			robot.semiAuto.turnToNearestAngle(ROCKET_SIDE_ANGLES);
			resetPlace = false;
		} else if (joy.getRawButton(10)) {
			robot.semiAuto.turnToNearestAngle(ROCKET_HATCH_ANGLES);
			resetPlace = false;
		} else {
			robot.semiAuto.reng = false;
		}

		robot.fieldOriented = !joy.getRawButton(2);

		if (xbox.getRawButton(7)) {
			robot.intake.setIntake(false);
		} else if (xbox.getRawButton(8)) {
			robot.intake.setIntake(true);
		} else {
			robot.intake.stopIntake();
		}

		if (leadingEdge(true, 1)) {
			robot.intake.toggleHatch();
		}

		if (leadingEdge(false, 9)) {
			robot.climb.reset(0);
		}

		leadingEdge(false, 10);//robot.elevator.reset(0);

		if (joy.getRawButton(3)) {
			robot.semiAuto.climb(0);
		} else if (joy.getRawButton(4)) {
			robot.semiAuto.climb(1);
		} else {
			robot.semiAuto.reset(false);
			if (robot.semiAuto.lift) {
				robot.elevator.moveTo(robot.semiAuto.pFrontPos);
				robot.climb.moveTo(robot.semiAuto.pBackPos);
				robot.semiAuto.lift = false;
			}
		}

		if (leadingEdge(false, 5)) {
			robot.semiAuto.elevatorSetpoint(TRAVEL_ANGLE, TRAVEL_HEIGHT, true);
			robot.intake.setHatch(false);
		} else if (leadingEdge(false, 6)) {
			robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, INTAKE_HEIGHT, true);
			robot.intake.setHatch(true);
		}


		if (xbox.getRawButton(1)) {
			robot.elevator.moveTo(PLACE_HEIGHT[0]);
		} else if (xbox.getRawButton(4)) {
			robot.elevator.moveTo(PLACE_HEIGHT[1]);
		} else if (xbox.getRawButton(3)) {
			robot.elevator.moveTo(PLACE_HEIGHT[2]);
			robot.intake.moveWristTo(HATCH_LEVEL_THREE_WRIST);
		} else if (xbox.getRawButton(2)) {
			robot.elevator.moveTo(CARGO_LOADING_STATION_HEIGHT);
			robot.intake.moveWristTo(CARGO_LOADING_STATION_ANGLE);
		}

		if (xbox.getPOV() == 0) {
			robot.intake.moveWristTo(CARGO_WRIST_ANGLE);
		} else if (xbox.getPOV() == 90) {
			robot.intake.moveWristTo(HATCH_WRIST_ANGLE);
		} else if (xbox.getPOV() == 270) {
			robot.intake.moveWristTo(WRIST_UP);
		} else if (xbox.getPOV() == 180) {
			robot.intake.moveWristTo(CARGO_WRIST_DOWN_ANGLE);
		}

		if (resetPlace) {
			robot.semiAuto.reset(true);
		}
	}

	//Returns true for the first frame the button is pressed
	private boolean leadingEdge(boolean joystick, int button) {
		int n = joystick ? 0 : 1;
		if (controllers[n].getRawButton(button)) {
			if (canPress[n][button - 1]) {
				canPress[n][button - 1] = false;
				return true;
			}
		} else {
			canPress[n][button - 1] = true;
		}
		return false;
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
