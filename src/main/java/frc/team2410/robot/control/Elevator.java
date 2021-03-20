package frc.team2410.robot.control;

import edu.wpi.first.wpilibj.Encoder;
import frc.team2410.robot.DashboardComponent;
import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;
import frc.team2410.robot.TalonPair;
import frc.team2410.robot.input.InputSource;
import frc.team2410.robot.input.StickAxis;
import frc.team2410.robot.input.StickPosition;

import java.util.HashMap;
import java.util.Map;

import static frc.team2410.robot.RobotMap.*;

public class Elevator implements LogicController, DashboardComponent {
	private Robot robot;

	private final Encoder heightEncoder;
	public TalonPair winchMotor;
	private double targetHeight;
	private double offset;
	private boolean checkStartReleased = false;

	@Override
	public String getDashboardName() {
		return "Elevator";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("A-Current", winchMotor.getACurrent());
		map.put("B-Current", winchMotor.getBCurrent());
		map.put("Position", getPosition());
		map.put("Target", getTarget());
		map.put("Bad Current", winchMotor.badCurrent());
		return map;
	}

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
		double elevatorStick = robot.inputManager.getAnalogStick(StickPosition.RIGHT, StickAxis.Y);
		if (robot.inputManager.getButtonState(InputSource.XBOX, 10)) {
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
