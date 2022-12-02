import frc.robot.subsystems.drives.Drives;
import frc.robot.utils.AutoUtil;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

public class AutoTest {
    static Drives drives;

    @BeforeAll
    public static void setup() {
        drives = new Drives();
    }

    @Test
    public void validateAutoPaths() {
        for(AutoUtil.Routine routine : AutoUtil.Routine.values()) {
            assertDoesNotThrow(() -> AutoUtil.generateCommand(routine.name(), drives));
        }
    }
}
