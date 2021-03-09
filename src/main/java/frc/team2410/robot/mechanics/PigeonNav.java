package frc.team2410.robot.mechanics;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import static frc.team2410.robot.RobotMap.PIGEON_IMU_SRX;

public class PigeonNav implements PIDSource {
	private final PigeonIMU gyro;
	private int offset;

	public PigeonNav() {
		this.gyro = new PigeonIMU(PIGEON_IMU_SRX);
		this.resetHeading(0);
	}

	@Override
	public double pidGet() {
		return this.getHeading();
	}

	public double getHeading() {
		double angle = (((this.gyro.getFusedHeading() - offset) % 360.0) + 360) % 360; // Wraps angle between -360:360, changes negative values to equivalent postive values (ex. -90 -> 270 degrees) (changing the range to 0:360)
		return angle <= 180 ? angle : angle - 360;  // Changes >180 Degrees to Neg Equivalent (ex. 270 -> -90) (changing the range to -180:180) and returns it
	}

	public void resetHeading(int head) {
		this.gyro.setFusedHeading(0, 20);
		this.offset = head;
	}

	public PigeonIMU.PigeonState getStatus() {
		return gyro.getState();
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
	}
}
