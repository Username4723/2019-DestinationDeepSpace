package frc.team2410.robot.control;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.team2410.robot.DashboardComponent;
import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;
import frc.team2410.robot.input.InputSource;

import java.util.HashMap;
import java.util.Map;

import static frc.team2410.robot.RobotMap.*;

public class Climb implements LogicController, DashboardComponent {
	private Robot robot;

	private final WPI_TalonSRX winchMotor;
	private final Encoder heightEncoder;

	private double targetHeight;
	private double offset;

	@Override
	public String getDashboardName() {
		return "Climb";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("Current", getCurrent());
		map.put("Position", getPosition());
		map.put("Target", getTarget());
		return map;
	}

	public Climb(Robot robot) {
		this.robot = robot;

		winchMotor = new WPI_TalonSRX(CLIMB_ELEVATOR);
		heightEncoder = new Encoder(CLIMB_ELEVATOR_A, CLIMB_ELEVATOR_B);
		winchMotor.setInverted(true);
		heightEncoder.setDistancePerPulse(WINCH_CLIMB_DIST_PER_PULSE);
		heightEncoder.reset();
		heightEncoder.setReverseDirection(true);
		targetHeight = heightEncoder.get();
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
		if (robot.inputManager.getPOV(InputSource.JOYSTICK) != 0 && robot.inputManager.getPOV(InputSource.JOYSTICK) != 180 && !robot.semiAuto.lift) {
			double speed = -(targetHeight - getPosition());
			if (speed > 0) speed /= 15;
			speed = Math.max(-1, Math.min(1, speed));
			winchMotor.set(speed);
		} else if (!robot.semiAuto.lift) {
			if (!(getPosition() < 0) || robot.inputManager.getPOV(InputSource.JOYSTICK) != 0)
				winchMotor.set(robot.inputManager.getPOV(InputSource.JOYSTICK) == 0 ? robot.inputManager.getSlider() : -robot.inputManager.getSlider());
			else winchMotor.set(0);
			targetHeight = getPosition();
		}
	}

	public double getCurrent() {
		return winchMotor.getOutputCurrent();
	}
}