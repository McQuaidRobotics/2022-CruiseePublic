import frc.robot.utils.UnitUtil;
import org.junit.jupiter.api.Test;

public class UnitUtilTest {
    @Test
    public void testRPMtoFaclon(){
        assert(UnitUtil.RPMToFalcon(5000, 2) == (5000.0*2.0)*(2048.0/600.0));
    }
}
