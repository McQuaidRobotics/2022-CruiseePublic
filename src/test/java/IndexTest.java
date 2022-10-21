import org.junit.*;

import edu.wpi.first.hal.HAL;
import frc.robot.commands.index.DefaultIndex;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.BallState;
import frc.robot.subsystems.Index.IndexState;

public class IndexTest {
    IndexState state;
    @Before
    public void Setup(){
        // assert HAL.initialize(500, 0);
        state = new IndexState(BallState.NONE);
    }
    @Test
    public void testBallsIndexed(){
        state.update(false);
        assert(state.getState() == BallState.NONE);
        assert(state.getDesiredState() == BallState.NONE);
        state.update(true);
        assert(state.getState() == BallState.BOTTOM);
        assert(state.getDesiredState() == BallState.TOP);
        state.update(true);
        assert(state.getState() == BallState.BOTTOM);
        assert(state.getDesiredState() == BallState.TOP);
        state.update(false);
        assert(state.getState() == BallState.TOP);
        assert(state.getDesiredState() == BallState.TOP);
        state.update(false);
        assert(state.getState() == BallState.TOP);
        assert(state.getDesiredState() == BallState.TOP);
        state.update(true);
        assert(state.getState() == BallState.BOTH);
        assert(state.getDesiredState() == BallState.BOTH);
        state.removeBall();
        assert(state.getState() == BallState.TOP);
        assert(state.getDesiredState() == BallState.TOP);
        state.removeBall();
        assert(state.getState() == BallState.NONE);
        assert(state.getDesiredState() == BallState.NONE);
    }
    @Test
    public void testCommandSequence(){
        
    }
}
