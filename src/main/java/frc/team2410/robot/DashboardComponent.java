package frc.team2410.robot;

import java.util.Map;

public interface DashboardComponent {
    String getDashboardName();
    Map<String, Object> getReportedData();
}
