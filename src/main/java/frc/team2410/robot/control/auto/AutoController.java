package frc.team2410.robot.control.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;

import static frc.team2410.robot.RobotMap.*;

public class AutoController implements LogicController {
    private Robot robot;

    public Timer timer;
    private AutoState autoState = AutoState.CARGOSHIP_LEFT;
    private int state = 0;

    public AutoController(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        state = 0;
        timer = new Timer();
    }

    public void loop() {
        SmartDashboard.putNumber("Auto Internal Substate", state);
        SmartDashboard.putString("Auto State", autoState.name());
        switch (autoState) {
            case CARGOSHIP_LEFT:
                cargoship(true);
                break;
            case CARGOSHIP_RIGHT:
                cargoship(false);
                break;
            case ROCKET_LEFT_FRONT:
                rocketFront(true);
                break;
            case ROCKET_RIGHT_FRONT:
                rocketFront(false);
        }
    }

    private void cargoship(boolean left) {
        final double STRAFE_SPEED = 0.1;

        switch (state) {
            case 0:
                timer.reset();
                timer.start();
                robot.drivetrain.desiredHeading = 0;
                state++;
                break;
            case 1:
                // Drive to cargoship
                robot.drivetrain.crabDrive(left ? -STRAFE_SPEED : STRAFE_SPEED, 0.8, 0, 1, true);
                if (timer.get() > 2.125) {
                    robot.drivetrain.brake();
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 2:
                // Raise elevator and extend wrist
                robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, PLACE_HEIGHT[0], true);
                if (timer.get() > 0.1) {
                    timer.reset();
                    timer.start();
                    //TODO Advance
                }
                break;
        }
    }

    private void rocketFront(boolean left) {
        final double X_SPEED = 0.80;

        switch (state) {
            case 0:
                timer.reset();
                timer.start();
                state++;
                break;
            case 1:
                // Drive off hab
                robot.drivetrain.crabDrive(0, 0.85, 0, 1, true);
                if (timer.get() > 1.6) {
                    robot.drivetrain.brake();
                    robot.drivetrain.desiredHeading = left ? ROCKET_LEFT_LEFT : ROCKET_RIGHT_RIGHT;
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 2:
                // Drive to rocket
                robot.drivetrain.crabDrive(left ? -X_SPEED : X_SPEED, 0.65, 0, 1, true);
                if (timer.get() > 2.125) {
                    robot.drivetrain.brake();
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 3:
                // Raise elevator and extend wrist
                robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, PLACE_HEIGHT[0], true);
                if (timer.get() > 0.1) {
                    timer.reset();
                    // TODO advance
                }
                break;
        }
    }
}