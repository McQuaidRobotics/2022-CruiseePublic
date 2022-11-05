import frc.robot.subsystems.Index.BallState;
import frc.robot.subsystems.Index.IndexState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IndexTest {
    static IndexState state;

    @BeforeAll
    public static void setup(){
        // assert HAL.initialize(500, 0);
        state = new IndexState(BallState.NONE);
        state.setTest(true);
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
}
