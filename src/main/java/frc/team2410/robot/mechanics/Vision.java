package frc.team2410.robot.mechanics;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team2410.robot.DashboardComponent;

import java.util.HashMap;
import java.util.Map;

public class Vision implements DashboardComponent {
	private final NetworkTable table;
	private Number[] centerX;
	private Number[] centerY;
	private Number[] area;

	@Override
	public String getDashboardName() {
		return "Vision";
	}

	@Override
	public Map<String, Object> getReportedData() {
		Map<String, Object> map = new HashMap<>();
		map.put("CenterX", getCentralValue()[0]);
		map.put("CenterY", getCentralValue()[1]);
		map.put("Line", getCentralValue()[0] != 0);
		return map;
	}

	public Vision() {
		table = NetworkTableInstance.getDefault().getTable("GRIP/AllDemContours");
		this.update();
	}

	private void update() {
		centerX = table.getEntry("centerX").getNumberArray(new Number[0]);
		centerY = table.getEntry("centerY").getNumberArray(new Number[0]);
		area = table.getEntry("area").getNumberArray(new Number[0]);
	}

	public double[] getCentralValue() {
		this.update();
		double x = 0;
		double y = 0;
		double greatestArea = 0;
		for (int i = 0; (i < area.length) && (i < centerX.length) && (i < centerY.length); i++) {
			if ((double) area[i] > greatestArea) {
				x = (double) centerX[i];
				y = (double) centerY[i];
				greatestArea = (double) area[i];
			}
		}
		return new double[]{x, y};
	}
}
