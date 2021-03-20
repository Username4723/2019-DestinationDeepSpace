package frc.team2410.robot.mechanics;

import edu.wpi.first.wpilibj.PIDController;
import frc.team2410.robot.DashboardComponent;
import frc.team2410.robot.GameState;
import frc.team2410.robot.NumericalPIDOutput;
import frc.team2410.robot.Robot;
import frc.team2410.robot.config.WheelPosition;

import java.util.HashMap;
import java.util.Map;

import static frc.team2410.robot.RobotMap.*;

public class Drivetrain implements DashboardComponent {
	private Robot robot;

	private final PIDController gyroPID;
	public double desiredHeading;
	public Map<WheelPosition, SwerveModule> swerveModules = new HashMap<>();
	private double pHead = 0; // Previous heading

	@Override
	public String getDashboardName() {
		return "Drivechain";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		for (WheelPosition position : swerveModules.keySet()) {
			Map<String, Object> moduleInfo = new HashMap<>();
			SwerveModule module = swerveModules.get(position);

			moduleInfo.put("Angle", module.getAngle());
			moduleInfo.put("Encoder Voltage", module.positionEncoder.getVoltage());

			map.put(position.name(), moduleInfo);
		}

		map.put("Desired Heading", wrap(desiredHeading, -180.0, 180.0));
		return map;
	}

	public Drivetrain(Robot robot) {
		this.robot = robot;

		for (WheelPosition wheelPosition : WheelPosition.values()) {
			swerveModules.put(wheelPosition, new SwerveModule(robot.config.WHEELS.get(wheelPosition)));
		}
		this.desiredHeading = robot.gyro.getHeading();
		//this.driveEnc = new Encoder(DRIVE_CIMCODER_A, DRIVE_CIMCODER_B);
		//this.driveEnc.setDistancePerPulse(DRIVE_DIST_PER_PULSE);\

		this.gyroPID = new PIDController(GYRO_P, GYRO_I, GYRO_D, robot.gyro, new NumericalPIDOutput(), 0.002);
		gyroPID.setInputRange(-180, 180);
		gyroPID.setOutputRange(-0.3, 0.3);
		gyroPID.setContinuous(true);
		gyroPID.enable();
	}

	public void resetHeading(int head) {
		robot.gyro.resetHeading(head);
		desiredHeading = head;
	}

	public void joystickDrive(boolean fieldOriented) {
		double speedMultiplier = robot.inputManager.getSlider();
		crabDrive(robot.inputManager.getX(), robot.inputManager.getY(), robot.inputManager.getTwist(), speedMultiplier, fieldOriented);
	}

	public void returnWheelsToZero() {
		swerveModules.values().forEach(SwerveModule::returnToZero);
	}

	public void brake() {
		swerveModules.values().forEach(it -> it.drive(0, 0));
	}

	public void crabDrive(double x, double y, double rotation, double speedMultiplier, boolean useGyro) {
		double forward, strafe;
		double heading = robot.gyro.getHeading() * Math.PI / 180; //Degrees -> Radians
		if (useGyro) {
			forward = -x * Math.sin(heading) + y * Math.cos(heading);
			strafe = x * Math.cos(heading) + y * Math.sin(heading);
		} else {
			forward = y;
			strafe = x;
		}

		// Sets desired heading dependant if gyro still moving
		if ((rotation == 0 && Math.abs(pHead - robot.gyro.getHeading()) < 1) || robot.semiAuto.reng || robot.currentState == GameState.AUTONOMOUS) {
			gyroPID.setSetpoint(desiredHeading);
			rotation = -gyroPID.get();
		} else {
			desiredHeading = robot.gyro.getHeading();
			desiredHeading = wrap(desiredHeading, 180, -180);
		}
		pHead = robot.gyro.getHeading();

		if (x != 0 || y != 0 || rotation != 0) {
			double back, front, right, left;
			if (rotation != 0) {
				back = strafe - rotation / Math.sqrt(2); //strafe-rotation*1.0/Math.sqrt(2);
				front = strafe + rotation / Math.sqrt(2);
				right = forward - rotation / Math.sqrt(2);
				left = forward + rotation / Math.sqrt(2);
			} else {
				back = strafe;
				front = strafe;
				right = forward;
				left = forward;
			}
			double flds = Math.sqrt(front * front + left * left);
			double frds = Math.sqrt(front * front + right * right);
			double blds = Math.sqrt(back * back + left * left);
			double brds = Math.sqrt(back * back + right * right);
			double maxSpeed = Math.max(Math.max(Math.max(flds, frds), blds), brds);
			if (maxSpeed > 1) {
				flds /= maxSpeed;
				frds /= maxSpeed;
				blds /= maxSpeed;
				brds /= maxSpeed;
			}
			double fla = 0, fra = 0, bla = 0, bra = 0;
			if (front != 0 || left != 0) {
				fla = (180 / Math.PI) * -Math.atan2(front, left);
				fla = wrap(fla, 360, 0);
			}
			if (front != 0 || right != 0) {
				fra = (180 / Math.PI) * -Math.atan2(front, right);
				fra = wrap(fra, 360, 0);
			}
			if (back != 0 || left != 0) {
				bla = (180 / Math.PI) * -Math.atan2(back, left);
				bla = wrap(bla, 360, 0);
			}
			if (back != 0 || right != 0) {
				bra = (180 / Math.PI) * -Math.atan2(back, right);
				bra = wrap(bra, 360, 0);
			}
			this.swerveModules.get(WheelPosition.FRONT_LEFT).drive(flds * speedMultiplier, fla);
			this.swerveModules.get(WheelPosition.FRONT_RIGHT).drive(frds * speedMultiplier, fra);
			this.swerveModules.get(WheelPosition.BACK_LEFT).drive(blds * speedMultiplier, bla);
			this.swerveModules.get(WheelPosition.BACK_RIGHT).drive(brds * speedMultiplier, bra);
		} else {
			swerveModules.values().forEach(it -> it.drive(0, it.getAngle()));
		}
	}

	public double wrap(double num, double max, double min) {
		return (num - min) - (max - min) * Math.floor((num - min) / (max - min)) + min;
	}
}