package frc.team2410.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2410.robot.Robot;
import static frc.team2410.robot.RobotMap.*;

public class SemiAuto {
	
	public int placeState = 0;
	private int climbState = 0;
	public Timer t;
	public boolean engaged = false;
	public boolean ceng = false;
	public boolean reng = false;
	public boolean lift = false;
	public double pval = 0;
	public double pFrontPos;
	public double pBackPos;
	public double targetAngle;
	
	
	public SemiAuto() {
		
		t = new Timer();
		targetAngle = Robot.gyro.getHeading();
	}
	
	public void reset(boolean place) {
		if(place) {
			placeState = 0;
			reng = false;
		} else {
			climbState = 0;
			ceng = false;
		}
		engaged = ceng || reng;
	}
	
	public boolean startMatch() {
		Robot.intake.moveWristTo(TRAVEL_ANGLE + ((WRIST_UP - TRAVEL_ANGLE) * (1-(t.get() / 2))));
		if(Math.abs(Robot.intake.getAngle() - TRAVEL_ANGLE) < 5) {
			Robot.intake.toggleHatch();
			t.stop();
			return true;
		}
		return false;
	}
	
	public void turnToNearestAngle(double angle) {
		reng = true;
		engaged = true;
		Robot.drivetrain.desiredHeading = angle;
	}
	
	public void turnToNearestAngle(double [] angles) {
		reng = true;
		engaged = true;
		
		double angle = Robot.gyro.getHeading();
		double lowestOffset = 180;
		double target = 0;

		for (double v : angles) {
			double offset = Math.abs(v - angle);
			if (offset < lowestOffset) {
				lowestOffset = offset;
				target = v;
			}
		}
		
		Robot.drivetrain.desiredHeading = target;
	}

	private void lift(int level) {
		Robot.climb.moveTo(CLIMB_HEIGHT[level] + CLIMB_ELEVATOR_MAX_OFFSET);
		
		if(elevatorSetpoint(CLIMB_WRIST_ANGLE[1], CLIMB_HEIGHT[level] - Robot.climb.getPosition(), true)) {
			//Robot.elevator.setIntake(false);
		}
	}
	
	public void climb(int level) {
		ceng = true;
		engaged = true;
		switch(climbState) {
			case 0:
				//Robot.climb.moveTo(CLIMB_OFFSET * 2);
				if(elevatorSetpoint(CLIMB_WRIST_ANGLE[0], CLIMB_HEIGHT[level], true)) {
					climbState++;
					t.reset();
					t.start();
				}
				break;
			case 1:
				if(t.get() > 1) {
					climbState++;
				}
				break;
			case 2:
				lift(level);
				break;
		}
	}
	
	public boolean elevatorSetpoint(double wristAngle, double elevatorHeight, boolean sameTime) {
		boolean elevatorAt = Math.abs(Robot.elevator.getPosition() - elevatorHeight) < 3;
		boolean wristAt = Math.abs(Robot.intake.getAngle() - wristAngle) < 5;
		if(elevatorAt || sameTime) Robot.intake.moveWristTo(wristAngle);
		Robot.elevator.moveTo(elevatorHeight);
		return elevatorAt && wristAt;
	}
}
