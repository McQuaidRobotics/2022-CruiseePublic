package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

public class RobotIdentifier {
    private static final Map<String, String> numToName = Map.of(
            "0306adf3", "testBoard",
            "0306adcf", "lance"
    );

    public static String getRobotName() {
        String robotName = numToName.get(RobotController.getSerialNumber());
        if(robotName == null) {
            robotName = "";
        }
        return robotName;
    }

    public static void initNetworkTables() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        instance.getEntry("/serialNum").setString(RobotController.getSerialNumber());
        instance.getEntry("/robotName").setString(getRobotName());

        try {
            Path deployDir = Filesystem.getDeployDirectory().toPath();
            Path branchFile = deployDir.resolve("branch.txt");
            Path commitFile = deployDir.resolve("commit.txt");

            instance.getEntry("/gitBranch").setString(Files.readString(branchFile));
            instance.getEntry("/gitCommit").setString(Files.readString(commitFile));
        } catch (IOException e) {
            System.err.println("Could not read git information files.");
        }
    }
}
