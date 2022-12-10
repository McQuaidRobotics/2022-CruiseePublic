package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Map;

public class RobotIdentifier {
    private static final Map<String, String> numToName = Map.of(
            "0306ADF3", "testBoard",
            "0306ADCF", "lance"
    );

    public static String getRobotName() {
        String robotName = numToName.get(RobotController.getSerialNumber());
        NetworkTableInstance.getDefault().getEntry("/robotName").setString(robotName);
        return robotName;
    }
}
