package frc.team2410.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2410.robot.control.Climb;
import frc.team2410.robot.control.Elevator;
import frc.team2410.robot.control.Intake;
import frc.team2410.robot.control.auto.AutoController;
import frc.team2410.robot.mechanics.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Robot extends TimedRobot {
	public static Drivetrain drivetrain;
	public static PigeonNav gyro;
	public static OI oi;
	public static Vision vision;
	public static SemiAuto semiAuto;
	public static Elevator elevator;
	public static Intake intake;
	public static Climb climb;
	public static LED led;
	static boolean fieldOriented = true;
	private boolean startMatch = true;

	private Map<GameState, List<LogicController>> gameControllers = new HashMap<>();
	public static GameState currentState;

	public Robot() {
	}

	@Override
	public void robotInit() {
		//Create subsystems
		gyro = new PigeonNav();
		drivetrain = new Drivetrain();
		oi = new OI();
		vision = new Vision();
		semiAuto = new SemiAuto();
		elevator = new Elevator();
		climb = new Climb();
		led = new LED();
		led.setColor(0, 0, 255);

		registerLogicController(GameState.AUTONOMOUS, new AutoController());
		registerLogicController(GameState.TELEOP, intake);
		registerLogicController(GameState.TELEOP, elevator);
		registerLogicController(GameState.TELEOP, climb);
	}

	private void registerLogicController(GameState state, LogicController controller) {
		this.gameControllers.computeIfAbsent(state, it -> new ArrayList<>()).add(controller);
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("FL Angle", drivetrain.fl.getAngle());
		SmartDashboard.putNumber("FR Angle", drivetrain.fr.getAngle());
		SmartDashboard.putNumber("BL Angle", drivetrain.bl.getAngle());
		SmartDashboard.putNumber("BR Angle", drivetrain.br.getAngle());
		SmartDashboard.putNumber("FL Voltage", drivetrain.fl.positionEncoder.getVoltage());
		SmartDashboard.putNumber("FR Voltage", drivetrain.fr.positionEncoder.getVoltage());
		SmartDashboard.putNumber("BL Voltage", drivetrain.bl.positionEncoder.getVoltage());
		SmartDashboard.putNumber("BR Voltage", drivetrain.br.positionEncoder.getVoltage());
		SmartDashboard.putNumber("ElevatorA Current", elevator.winchMotor.getAcurrent());
		SmartDashboard.putNumber("ElevatorB Current", elevator.winchMotor.getBcurrent());
		SmartDashboard.putNumber("Wrist Current", intake.getWristCurrent());
		SmartDashboard.putNumber("Climb Current", climb.getVoltage());
		SmartDashboard.putNumber("Wrist Voltage", intake.getVoltage());
		SmartDashboard.putNumber("CenterX", vision.getCentralValue()[0]);
		SmartDashboard.putNumber("CenterY", vision.getCentralValue()[1]);
		SmartDashboard.putNumber("Heading", gyro.getHeading());
		SmartDashboard.putString("Gyro Status", gyro.getStatus().toString());
		SmartDashboard.putNumber("Desired Heading", drivetrain.wrap(drivetrain.desiredHeading, -180.0, 180.0));
		SmartDashboard.putNumber("Wrist Angle", intake.getAngle());
		SmartDashboard.putNumber("Wrist Target", intake.getWristTarget());

		SmartDashboard.putNumber("Elevator height", elevator.getPosition());
		SmartDashboard.putNumber("Elevator target", elevator.getTarget());
		SmartDashboard.putNumber("Climb height", climb.getPosition());
		SmartDashboard.putNumber("Climb target", climb.getTarget());
		SmartDashboard.putNumber("Place State", semiAuto.placeState);
		SmartDashboard.putNumber("R", led.r);
		SmartDashboard.putNumber("G", led.g);
		SmartDashboard.putNumber("B", led.b);
		SmartDashboard.putBoolean("Toasty Elevator", !elevator.winchMotor.badCurrent());
		SmartDashboard.putBoolean("Hatch Intake Status", intake.getHatchStatus());
		SmartDashboard.putBoolean("Semi-Auto Done", semiAuto.placeState == -1);
		SmartDashboard.putBoolean("Line", vision.getCentralValue()[0] != 0);
		SmartDashboard.putBoolean("Semiauto Engaged", semiAuto.engaged);
	}

	@Override
	public void disabledInit() {
		led.setColor(255, 0, 0);
	}

	@Override
	public void disabledPeriodic() {
		led.fade(3);
	}

	@Override
	public void autonomousInit() {
		elevator.reset(0);
		led.setColor(0, 0, 255);
		startMatch = true;
		semiAuto.t.reset();
		semiAuto.t.start();
		currentState = GameState.AUTONOMOUS;
		gameControllers.get(GameState.AUTONOMOUS).forEach(LogicController::init);
	}

	@Override
	public void autonomousPeriodic() {
		if (startMatch) {
			startMatch = !semiAuto.startMatch();
		}

		gameControllers.get(GameState.AUTONOMOUS).forEach(LogicController::loop);
		elevator.autoLoop();
	}

	@Override
	public void teleopInit() {
		currentState = GameState.TELEOP;
	}

	@Override
	public void teleopPeriodic() {
		//Run subsystem loops
		oi.pollButtons();
		gameControllers.get(GameState.AUTONOMOUS).forEach(LogicController::loop);
		drivetrain.joystickDrive(fieldOriented);

		if (elevator.winchMotor.badCurrent()) {
			led.status(255, 255, 0, 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()), fieldOriented);
		} else if (semiAuto.placeState == -1) {
			led.status(255, 0, 255, 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()), fieldOriented);
		} else if (semiAuto.engaged) {
			led.status(255, 0, 0, 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()), fieldOriented);
		} else if (vision.getCentralValue()[0] != 0) {
			led.status(0, 255, 0, 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()), fieldOriented);
		} else {
			led.status(0, 0, 255, 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()), fieldOriented);
		}

		SmartDashboard.putNumber("LED Speed", 10 + (int) (10 * Math.sqrt(oi.getX() * oi.getX() + oi.getY() * oi.getY()) * oi.getSlider()));
	}
}