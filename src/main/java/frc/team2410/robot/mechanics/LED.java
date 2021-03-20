package frc.team2410.robot.mechanics;

import edu.wpi.first.wpilibj.SerialPort;
import frc.team2410.robot.DashboardComponent;
import net.bak3dnet.robotics.led.LightDrive12;
import net.bak3dnet.robotics.led.modules.AStaticColorModule;

import java.util.HashMap;
import java.util.Map;

public class LED implements DashboardComponent {
	private final LightDrive12 controller = new LightDrive12(SerialPort.Port.kMXP);
	public double r;
	public double g;
	public double b;
	private int state = 0;

	@Override
	public String getDashboardName() {
		return "LED";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("Red", r);
		map.put("Green", g);
		map.put("Blue", b);
		map.put("State", state);
		return map;
	}

	public void setColor(double r, double g, double b) {
		this.r = r;
		this.g = g;
		this.b = b;
		if (r > 255) r = 255;
		if (g > 255) g = 255;
		if (b > 255) b = 255;
		if (r < 0) r = 0;
		if (g < 0) g = 0;
		if (b < 0) b = 0;
		AStaticColorModule c = new AStaticColorModule(new byte[]{(byte) r, (byte) b, (byte) g});
		controller.setChannelModule(1, c);
		controller.setChannelModule(2, c);
		controller.setChannelModule(3, c);
		controller.setChannelModule(4, c);
	}

	public void blink(int r0, int g0, int b0, int r1, int g1, int b1, int speed) {
		state += speed;
		if (state > 255 / 2) {
			state = 0;
			if (r0 == r && g0 == g && b0 == b) {
				setColor(r1, g1, b1);
			} else {
				setColor(r0, g0, b0);
			}
		}
	}

	public void status(int r0, int g0, int b0, int speed, boolean status) {
		if (status) {
			setColor(r0, g0, b0);
		} else {
			blink(r0, g0, b0, 0, 0, 0, speed);
		}
	}
}