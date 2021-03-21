package frc.team2410.robot.control;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.team2410.robot.DashboardComponent;
import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;
import frc.team2410.robot.input.StickAxis;
import frc.team2410.robot.input.StickPosition;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static frc.team2410.robot.RobotMap.*;

public class Intake implements LogicController, DashboardComponent {
	private Robot robot;

	private final DoubleSolenoid solenoid;
	private final WPI_TalonSRX wrist;
	private final AnalogInput wristEncoder;
	private final PIDController pid;
	WPI_TalonSRX wheels;
	private boolean open = true;

	@Override
	public String getDashboardName() {
		return "Intake";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("Wrist Voltage", getVoltage());
		map.put("Wrist Current", getWristCurrent());
		map.put("Wrist Angle", getAngle());
		map.put("Wrist Target", getWristTarget());
		map.put("Hatch Status", getHatchStatus());
		return map;
	}

	public Intake(Robot robot) {
		this.robot = robot;

		wheels = new WPI_TalonSRX(INTAKE_MOTOR);
		solenoid = new DoubleSolenoid(PCM, HATCH_INTAKE_FORWARD, HATCH_INTAKE_REVERSE);
		wrist = new WPI_TalonSRX(WRIST_MOTOR);
		wheels.setInverted(true);
		wrist.setInverted(true);
		wristEncoder = new AnalogInput(WRIST_ENCODER);
		pid = new PIDController(WRIST_P, WRIST_I, WRIST_D, 0.002); //  wristEncoder, wrist,
		pid.setTolerance(0.25 * 0.01);
		pid.enableContinuousInput(0, 5);

		pid.setSetpoint(getVoltage());
	}

	// intake section
	public void setIntake(boolean in) {
		wheels.set(in ? 1 : -1);
	}

	public void stopIntake() {
		wheels.set(0);
	}

	// hatch section
	public void setHatch(boolean open) {
		solenoid.set(!open ? kForward : kReverse);
	}

	public void toggleHatch() {
		open = !open;
		setHatch(open);
	}

	public boolean getHatchStatus() {
		return open;
	}

	// wrist section
	void setWrist(double speed) {
		wrist.set(speed);
	}

	public void moveWristTo(double angle) {
		pid.setSetpoint(getVoltage(angle));
	}

	public double getWristTarget() {
		return getAngle(pid.getSetpoint());
	}

	public double getAngle() {
		return getAngle(getVoltage());
	}

	public double getAngle(double voltage) {
		double angle = ((((voltage - WRIST_OFFSET) * (360.0 / 5) * (40.0 / 24)) % 360.0) + 360) % 360; // Wraps angle between -360:360, changes negative values to equivalent postive values (ex. -90 -> 270 degrees) (changing the range to 0:360)
		return angle <= 180 ? angle : angle - 360;  // Changes >180 Degrees to Neg Equivalent (ex. 270 -> -90) (changing the range to -180:180) and returns it
	}

	public double getVoltage(double angle) {
		return angle / ((360.0 / 5) * (40.0 / 24)) + WRIST_OFFSET;
	}

	public double getVoltage() {
		return wristEncoder.getVoltage();
	}

	public double getWristCurrent() {
		return wrist.getOutputCurrent();
	}

	public void loop() {
		double wristStick = robot.inputManager.getAnalogStick(StickPosition.LEFT, StickAxis.Y);
		if (wristStick != 0) {
			wrist.pidWrite(0);
			setWrist(-wristStick);
			pid.setSetpoint(getVoltage());
		} else {
			wrist.pidWrite(Math.max(-1, Math.min(1, pid.calculate(wristEncoder.pidGet()))));
		}
	}
}