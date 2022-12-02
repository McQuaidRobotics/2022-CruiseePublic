import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.kAuto;
import frc.robot.subsystems.drives.Drives;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class DrivesTest {
    static Drives drives;

    @BeforeAll
    public static void setup() {
        drives = new Drives();
    }

    @Test
    public void testPoseReset() {
        List<Pose2d> testPoses = List.of(
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(10)),
                new Pose2d(new Translation2d(8, 0), Rotation2d.fromDegrees(0)),
                new Pose2d(new Translation2d(0, 8), Rotation2d.fromDegrees(0)),
                new Pose2d(new Translation2d(10, 5), Rotation2d.fromDegrees(170))
        );

        for (Pose2d pose : testPoses) {
            drives.setOdometryPose(pose);
            assertEquals(pose, drives.getPose());
            assertEquals(pose.getRotation(), drives.getRotation());
        }
    }

    @Test
    public void validateAutoPaths() {
        for(kAuto.Routine routine : kAuto.Routine.values()) {
            assertDoesNotThrow(() -> drives.runAutoPath(routine.name()));
        }
    }
}
