import org.junit.Before;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.Acquisition;

/** Add your docs here. */
public class AcquisitionTest {
    Acquisition acq;
    
    @Before
    public void setup(){
        assert HAL.initialize(500, 0);
        acq = new Acquisition();
        
    }
}
