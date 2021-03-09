package frc.team2410.robot.control.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2410.robot.Robot;

import static frc.team2410.robot.RobotMap.*;

public class AutoController {
    public Timer timer;
    private AutoState autoState;
    private int state = 0;
    private boolean autoDone;

    public void init(AutoState autoState) {
        autoDone = false;
        state = 0;
        this.autoState = autoState;
        timer = new Timer();
    }

    public boolean getAutoDone() {
        return autoDone;
    }

    public void setAutoDone(boolean autoDone) {
        this.autoDone = autoDone;
    }

    public void loop() {
        SmartDashboard.putNumber("Auto Internal Substate", state);
        SmartDashboard.putString("Auto State", autoState.name());
        switch (autoState) {
            case DISABLED:
                autoDone = true;
                break;
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

        if (Robot.oi.getAbortAuto()) autoDone = true;
    }

    private void cargoship(boolean left) {
        final double STRAFE_SPEED = 0.1;

        switch (state) {
            case 0:
                timer.reset();
                timer.start();
                Robot.drivetrain.desiredHeading = 0;
                state++;
                break;
            case 1:
                // Drive to cargoship
                Robot.drivetrain.crabDrive(left ? -STRAFE_SPEED : STRAFE_SPEED, 0.8, 0, 1, true);
                if (timer.get() > 2.125) {
                    Robot.drivetrain.brake();
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 2:
                // Raise elevator and extend wrist
                Robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, PLACE_HEIGHT[0], true);
                if (timer.get() > 0.1) {
                    timer.reset();
                    timer.start();
                    autoDone = true;
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
                Robot.drivetrain.crabDrive(0, 0.85, 0, 1, true);
                if (timer.get() > 1.6) {
                    Robot.drivetrain.brake();
                    Robot.drivetrain.desiredHeading = left ? ROCKET_LEFT_LEFT : ROCKET_RIGHT_RIGHT;
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 2:
                // Drive to rocket
                Robot.drivetrain.crabDrive(left ? -X_SPEED : X_SPEED, 0.65, 0, 1, true);
                if (timer.get() > 2.125) {
                    Robot.drivetrain.brake();
                    timer.reset();
                    timer.start();
                    state++;
                }
                break;
            case 3:
                // Raise elevator and extend wrist
                Robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, PLACE_HEIGHT[0], true);
                if (timer.get() > 0.1) {
                    timer.reset();
                    autoDone = true;
                }
                break;
        }
    }
}