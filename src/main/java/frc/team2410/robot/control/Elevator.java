package frc.team2410.robot.control;

import edu.wpi.first.wpilibj.Encoder;
import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;
import frc.team2410.robot.TalonPair;

import static frc.team2410.robot.RobotMap.*;

public class Elevator implements LogicController {
	private Robot robot;

	private final Encoder heightEncoder;
	public TalonPair winchMotor;
	private double targetHeight;
	private double offset;
	private boolean checkStartReleased = false;


	public Elevator(Robot robot) {
		this.robot = robot;

		winchMotor = new TalonPair(ELEVATOR_A, ELEVATOR_B, true, false);
		heightEncoder = new Encoder(ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B);
		heightEncoder.setDistancePerPulse(WINCH_DIST_PER_PULSE);
		heightEncoder.reset();
	}

	public void moveTo(double height) {
		targetHeight = height;
	}

	public double getPosition() {
		return heightEncoder.getDistance() + offset;
	}

	public double getTarget() {
		return targetHeight;
	}

	public void reset(double height) {
		heightEncoder.reset();
		offset = height;
		targetHeight = height;
	}

	public void loop() {
		double elevatorStick = robot.userInput.getAnalogStick(true, true);
		if (robot.userInput.startPressed()) {
			winchMotor.set(0.2);
			checkStartReleased = true;
		} else if (checkStartReleased) {
			winchMotor.set(0);
			reset(0);
			checkStartReleased = false;
		} else if (elevatorStick == 0 && !robot.semiAuto.lift) {
			double speed = -((targetHeight - getPosition()) / 4.50);
			if (speed > 0 && !robot.semiAuto.ceng) speed /= 10.0;
			if (speed < -1) speed = -1;
			if (speed > 1) speed = 1;
			winchMotor.set(speed);
		} else if (!robot.semiAuto.lift && !(getPosition() < 0.5 && elevatorStick > 0) && !(getPosition() > 60 && elevatorStick < 0)) {
			winchMotor.set(elevatorStick);
			targetHeight = getPosition();
		}
	}

	public void autoLoop() {
		double speed = -((targetHeight - getPosition()) / 4.50);
		if (speed > 0) speed /= 10.0;
		if (speed < -1) speed = -1;
		if (speed > 1) speed = 1;
		winchMotor.set(speed);
	}
}
