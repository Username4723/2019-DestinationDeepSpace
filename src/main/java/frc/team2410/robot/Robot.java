package frc.team2410.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2410.robot.config.RobotConfig;
import frc.team2410.robot.control.Climb;
import frc.team2410.robot.control.Elevator;
import frc.team2410.robot.control.Intake;
import frc.team2410.robot.control.auto.AutoController;
import frc.team2410.robot.control.teleop.TeleOpController;
import frc.team2410.robot.input.InputManager;
import frc.team2410.robot.mechanics.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Robot extends TimedRobot {
	public Drivetrain drivetrain;
	public PigeonNav gyro;
	public InputManager inputManager;
	public Vision vision;
	public SemiAuto semiAuto;
	public Elevator elevator;
	public Intake intake;
	public Climb climb;
	public LED led;
	public boolean fieldOriented = true;
	private boolean startMatch = true;

	private Map<GameState, List<LogicController>> gameControllers = new HashMap<>();
	private List<DashboardComponent> dashboardComponents = new ArrayList<>();
	public GameState currentState;

	public RobotConfig config;

	public Robot(RobotConfig config) {
		this.config = config;
	}

	@Override
	public void robotInit() {
		//Create subsystems
		gyro = new PigeonNav();
		drivetrain = new Drivetrain(this);
		inputManager = config.createInputManager();
		vision = new Vision();
		semiAuto = new SemiAuto(this);
		elevator = new Elevator(this);
		climb = new Climb(this);
		led = new LED();
		led.setColor(0, 0, 255);

		registerLogicController(GameState.AUTONOMOUS, new AutoController(this));
		registerLogicController(GameState.TELEOP, new TeleOpController(this));
		registerLogicController(GameState.TELEOP, intake);
		registerLogicController(GameState.TELEOP, elevator);
		registerLogicController(GameState.TELEOP, climb);
		registerLogicController(GameState.ALL, drivetrain);

		registerDashboardComponent(led);
		registerDashboardComponent(semiAuto);
		registerDashboardComponent(vision);
		registerDashboardComponent(gyro);
	}

	private void registerLogicController(GameState state, LogicController controller) {
		this.gameControllers.computeIfAbsent(state, it -> new ArrayList<>()).add(controller);
		if (controller instanceof DashboardComponent) dashboardComponents.add((DashboardComponent) controller);
	}

	private void registerDashboardComponent(DashboardComponent component) {
		dashboardComponents.add(component);
	}

	public void publishData(String header, Map<String, Object> data) {
		for (Map.Entry<String, Object> entry : data.entrySet()) {
			String key = header + " - " + entry.getKey();
			Object value = entry.getValue();
			if (value instanceof Number) SmartDashboard.putNumber(key, ((Number) value).doubleValue());
			else if (value instanceof Map) publishData(key + " - ", (Map<String, Object>) value);
			else SmartDashboard.putString(key, value.toString());
		}
	}

	@Override
	public void robotPeriodic() {
		gameControllers.get(GameState.ALL).forEach(LogicController::loop);
		for (DashboardComponent component : dashboardComponents) {
			publishData(component.getDashboardName(), component.getReportedData());
		}
	}

	@Override
	public void disabledInit() {
		led.setColor(255, 0, 0);
	}

	@Override
	public void disabledPeriodic() {
		led.setColor(255, 0, 0);
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
		gameControllers.get(GameState.AUTONOMOUS).forEach(LogicController::loop);

		int speed = 10 + (int) (10 * Math.sqrt(Math.pow(inputManager.getX(), 2) + Math.pow(inputManager.getY(), 2)) * inputManager.getSlider());
		if (elevator.winchMotor.badCurrent()) {
			led.status(255, 255, 0, speed, fieldOriented);
		} else if (semiAuto.placeState == -1) {
			led.status(255, 0, 255, speed, fieldOriented);
		} else if (semiAuto.engaged) {
			led.status(255, 0, 0, speed, fieldOriented);
		} else if (vision.getCentralValue()[0] != 0) {
			led.status(0, 255, 0, speed, fieldOriented);
		} else {
			led.status(0, 0, 255, speed, fieldOriented);
		}

		SmartDashboard.putNumber("LED Speed", speed);
	}
}