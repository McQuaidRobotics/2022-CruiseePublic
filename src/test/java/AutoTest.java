import frc.robot.constants.kAuto;
import frc.robot.subsystems.drives.Drives;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

public class AutoTest {
    static Drives drives;

    @BeforeAll
    public static void setup() {
        drives = new Drives();
    }

    @Test
    public void validateAutoPaths() {
        for(kAuto.Routine routine : kAuto.Routine.values()) {
            assertDoesNotThrow(() -> drives.runAutoPath(routine.name()));
        }
    }
}
