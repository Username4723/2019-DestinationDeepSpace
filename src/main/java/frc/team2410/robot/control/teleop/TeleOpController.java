package frc.team2410.robot.control.teleop;

import frc.team2410.robot.LogicController;
import frc.team2410.robot.Robot;
import frc.team2410.robot.input.InputSource;

import static frc.team2410.robot.RobotMap.*;
import static frc.team2410.robot.input.InputSource.XBOX;

public class TeleOpController implements LogicController {
    private Robot robot;

    public TeleOpController(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void loop() {
        if (robot.inputManager.getButtonState(InputSource.JOYSTICK, 5)) {
            robot.drivetrain.returnWheelsToZero();
        }

        if (robot.inputManager.getButtonState(InputSource.JOYSTICK,6)) {
            robot.drivetrain.resetHeading(0);
        } else if (robot.inputManager.getButtonState(InputSource.JOYSTICK,7)) {
            robot.drivetrain.resetHeading(180);
        }

        boolean resetPlace = true;

        if (robot.inputManager.getButtonState(InputSource.JOYSTICK,11)) {
            robot.semiAuto.turnToNearestAngle(180);
            resetPlace = false;
        } else if (robot.inputManager.getButtonState(InputSource.JOYSTICK,9)) {
            robot.semiAuto.turnToNearestAngle(0);
            resetPlace = false;
        } else if (robot.inputManager.getButtonState(InputSource.JOYSTICK,12)) {
            robot.semiAuto.turnToNearestAngle(ROCKET_SIDE_ANGLES);
            resetPlace = false;
        } else if (robot.inputManager.getButtonState(InputSource.JOYSTICK,10)) {
            robot.semiAuto.turnToNearestAngle(ROCKET_HATCH_ANGLES);
            resetPlace = false;
        } else {
            robot.semiAuto.reng = false;
        }

        robot.fieldOriented = !robot.inputManager.getButtonState(InputSource.JOYSTICK,2);

        if (robot.inputManager.getButtonState(XBOX,7)) {
            robot.intake.setIntake(false);
        } else if (robot.inputManager.getButtonState(XBOX,8)) {
            robot.intake.setIntake(true);
        } else {
            robot.intake.stopIntake();
        }

        if (robot.inputManager.getLeadingButtonState(InputSource.JOYSTICK, 1)) {
            robot.intake.toggleHatch();
        }

        if (robot.inputManager.getLeadingButtonState(XBOX, 9)) {
            robot.climb.reset(0);
        }

        robot.inputManager.getLeadingButtonState(InputSource.JOYSTICK, 10);//robot.elevator.reset(0);

        if (robot.inputManager.getButtonState(InputSource.JOYSTICK,3)) {
            robot.semiAuto.climb(0);
        } else if (robot.inputManager.getButtonState(InputSource.JOYSTICK,4)) {
            robot.semiAuto.climb(1);
        } else {
            robot.semiAuto.reset(false);
            if (robot.semiAuto.lift) {
                robot.elevator.moveTo(robot.semiAuto.pFrontPos);
                robot.climb.moveTo(robot.semiAuto.pBackPos);
                robot.semiAuto.lift = false;
            }
        }

        if (robot.inputManager.getLeadingButtonState(XBOX, 5)) {
            robot.semiAuto.elevatorSetpoint(TRAVEL_ANGLE, TRAVEL_HEIGHT, true);
            robot.intake.setHatch(false);
        } else if (robot.inputManager.getLeadingButtonState(XBOX, 6)) {
            robot.semiAuto.elevatorSetpoint(HATCH_WRIST_ANGLE, INTAKE_HEIGHT, true);
            robot.intake.setHatch(true);
        }


        if (robot.inputManager.getButtonState(XBOX,1)) {
            robot.elevator.moveTo(PLACE_HEIGHT[0]);
        } else if (robot.inputManager.getButtonState(XBOX,4)) {
            robot.elevator.moveTo(PLACE_HEIGHT[1]);
        } else if (robot.inputManager.getButtonState(XBOX,3)) {
            robot.elevator.moveTo(PLACE_HEIGHT[2]);
            robot.intake.moveWristTo(HATCH_LEVEL_THREE_WRIST);
        } else if (robot.inputManager.getButtonState(XBOX,2)) {
            robot.elevator.moveTo(CARGO_LOADING_STATION_HEIGHT);
            robot.intake.moveWristTo(CARGO_LOADING_STATION_ANGLE);
        }

        int pov = robot.inputManager.getPOV(XBOX);
        if (pov == 0) {
            robot.intake.moveWristTo(CARGO_WRIST_ANGLE);
        } else if (pov == 90) {
            robot.intake.moveWristTo(HATCH_WRIST_ANGLE);
        } else if (pov == 270) {
            robot.intake.moveWristTo(WRIST_UP);
        } else if (pov == 180) {
            robot.intake.moveWristTo(CARGO_WRIST_DOWN_ANGLE);
        }

        if (resetPlace) {
            robot.semiAuto.reset(true);
        }
    }
}
