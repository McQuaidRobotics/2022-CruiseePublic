import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drives.Drives;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DrivesTest {
    static Drives drives;

    @BeforeAll
    public static void setup() {
        drives = new Drives();
    }

    @Test
    public void testPoseReset() {
        Pose2d pose = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(10));
        drives.setOdometryPose(pose);

        assertEquals(pose, drives.getPose());
    }
}
