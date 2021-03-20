package frc.team2410.robot.mechanics;

import edu.wpi.first.wpilibj.Timer;
import frc.team2410.robot.DashboardComponent;
import frc.team2410.robot.Robot;

import java.util.HashMap;
import java.util.Map;

import static frc.team2410.robot.RobotMap.*;

public class SemiAuto implements DashboardComponent {
	private Robot robot;

	public int placeState = 0;
	public Timer t;
	public boolean engaged = false;
	public boolean ceng = false;
	public boolean reng = false;
	public boolean lift = false;
	public double pFrontPos;
	public double pBackPos;
	public double targetAngle;
	private int climbState = 0;

	@Override
	public String getDashboardName() {
		return "Semi-Auto";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("Place State", placeState);
		map.put("Semi-Auto Done", placeState == -1);
		map.put("Semiauto Engaged", engaged);

		return map;
	}

	public SemiAuto(Robot robot) {
		this.robot = robot;

		t = new Timer();
		targetAngle = robot.gyro.getHeading();
	}

	public void reset(boolean place) {
		if (place) {
			placeState = 0;
			reng = false;
		} else {
			climbState = 0;
			ceng = false;
		}
		engaged = ceng || reng;
	}

	public boolean startMatch() {
		robot.intake.moveWristTo(TRAVEL_ANGLE + ((WRIST_UP - TRAVEL_ANGLE) * (1 - (t.get() / 2))));
		if (Math.abs(robot.intake.getAngle() - TRAVEL_ANGLE) < 5) {
			robot.intake.toggleHatch();
			t.stop();
			return true;
		}
		return false;
	}

	public void turnToNearestAngle(double angle) {
		reng = true;
		engaged = true;
		robot.drivetrain.desiredHeading = angle;
	}

	public void turnToNearestAngle(double[] angles) {
		reng = true;
		engaged = true;

		double angle = robot.gyro.getHeading();
		double lowestOffset = 180;
		double target = 0;

		for (double v : angles) {
			double offset = Math.abs(v - angle);
			if (offset < lowestOffset) {
				lowestOffset = offset;
				target = v;
			}
		}

		robot.drivetrain.desiredHeading = target;
	}

	private void lift(int level) {
		robot.climb.moveTo(CLIMB_HEIGHT[level] + CLIMB_ELEVATOR_MAX_OFFSET);

		if (elevatorSetpoint(CLIMB_WRIST_ANGLE[1], CLIMB_HEIGHT[level] - robot.climb.getPosition(), true)) {
			//Robot.elevator.setIntake(false);
		}
	}

	public void climb(int level) {
		ceng = true;
		engaged = true;
		switch (climbState) {
			case 0:
				//Robot.climb.moveTo(CLIMB_OFFSET * 2);
				if (elevatorSetpoint(CLIMB_WRIST_ANGLE[0], CLIMB_HEIGHT[level], true)) {
					climbState++;
					t.reset();
					t.start();
				}
				break;
			case 1:
				if (t.get() > 1) {
					climbState++;
				}
				break;
			case 2:
				lift(level);
				break;
		}
	}

	public boolean elevatorSetpoint(double wristAngle, double elevatorHeight, boolean sameTime) {
		boolean elevatorAt = Math.abs(robot.elevator.getPosition() - elevatorHeight) < 3;
		boolean wristAt = Math.abs(robot.intake.getAngle() - wristAngle) < 5;
		if (elevatorAt || sameTime) robot.intake.moveWristTo(wristAngle);
		robot.elevator.moveTo(elevatorHeight);
		return elevatorAt && wristAt;
	}
}
