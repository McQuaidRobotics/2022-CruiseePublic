package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class kAuto {
    private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, kSwerve.MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND);
    public static final PIDController THETA_AUTO_PID = new PIDController(5, 0.1, 0);
    public static final ProfiledPIDController THETA_AIMING_PID = new ProfiledPIDController(7.5, 0.1, 0, CONSTRAINTS, 0.02);

    static {
        THETA_AIMING_PID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public static final double XY_P = 3;

    public static final PIDController X_PID_CONTROLLER = new PIDController(XY_P, 0, 0);
    public static final PIDController Y_PID_CONTROLLER = new PIDController(XY_P, 0, 0);

    public enum Routine {
        HANGAR_TWO_BALL,
        TERMINAL_TWO_BALL,
        POTATO,
        NOTHING,
        DEFAULT,
        TEST
    }
}
