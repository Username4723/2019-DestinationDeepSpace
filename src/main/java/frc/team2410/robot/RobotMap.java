package frc.team2410.robot;

public class RobotMap {

	//PID
	public static final float SWERVE_MODULE_P = 5;
	public static final float SWERVE_MODULE_I = 0;
	public static final float SWERVE_MODULE_D = 3;
	public static final double GYRO_P = .03;
	public static final double GYRO_I = 0;
	public static final double GYRO_D = 0;
	public static final double WRIST_P = 5;
	public static final double WRIST_I = 0;
	public static final double WRIST_D = 3;

	//CAN
	public static final int PIGEON_IMU_SRX = 10;
	public static final int ELEVATOR_A = 9;
	public static final int ELEVATOR_B = 15;
	public static final int INTAKE_MOTOR = 11;
	public static final int WRIST_MOTOR = 12;
	public static final int PCM = 13;
	public static final int CLIMB_ELEVATOR = 14;

	//Analog In
	public static final int WRIST_ENCODER = 4;

	//DIO
	public static final int ELEVATOR_ENCODER_A = 8;
	public static final int ELEVATOR_ENCODER_B = 9;
	public static final int CLIMB_ELEVATOR_A = 0;
	public static final int CLIMB_ELEVATOR_B = 1;

	//PCM
	public static final int HATCH_INTAKE_FORWARD = 1;
	public static final int HATCH_INTAKE_REVERSE = 0;

	//Offsets
	public static final float WRIST_OFFSET_COMP = 4.06494099f;
	public static final float WRIST_OFFSET = WRIST_OFFSET_COMP;

	//Elevator Heights
	public static final double TRAVEL_HEIGHT = 2;
	public static final double INTAKE_HEIGHT = 5;
	public static final double CARGO_LOADING_STATION_HEIGHT = 30;
	public static final double[] PLACE_HEIGHT = {9, 36, 60};
	public static final double[] CLIMB_HEIGHT = {9, 22};
	public static final double CLIMB_ELEVATOR_MAX_OFFSET = 3.5;

	//Wrist Angles
	public static final double CARGO_WRIST_ANGLE = 55;
	public static final double CARGO_WRIST_DOWN_ANGLE = -5;
	public static final double HATCH_WRIST_ANGLE = 5;
	public static final double HATCH_LEVEL_THREE_WRIST = 14;
	public static final double CARGO_LOADING_STATION_ANGLE = 30;
	public static final double[] CLIMB_WRIST_ANGLE = {0, -20};
	public static final double WRIST_UP = 85;
	public static final double TRAVEL_ANGLE = 50;

	//Field Angles
	public static final double ROCKET_RIGHT_FRONT = -90;
	public static final double ROCKET_RIGHT_RIGHT = -90.0 + 61.25;
	public static final double ROCKET_RIGHT_LEFT = -90 - 61.25;
	public static final double ROCKET_LEFT_FRONT = 90;
	public static final double ROCKET_LEFT_RIGHT = 90.0 + 61.25;
	public static final double ROCKET_LEFT_LEFT = 90 - 61.25;
	public static final double[] ROCKET_SIDE_ANGLES = {ROCKET_RIGHT_FRONT, ROCKET_LEFT_FRONT};
	public static final double[] ROCKET_HATCH_ANGLES = {ROCKET_LEFT_LEFT, ROCKET_LEFT_RIGHT, ROCKET_RIGHT_LEFT, ROCKET_RIGHT_RIGHT};

	//Encoder Conversions
	//Diameter * PI / gear ratio / full encoder cycles (edges/4)
	public static final double WINCH_DIST_PER_PULSE = 1.91 * Math.PI * 2 / 65 / 3; //two stages
	public static final double WINCH_CLIMB_DIST_PER_PULSE = 2.00 * Math.PI / 216.66 / 3;
}
