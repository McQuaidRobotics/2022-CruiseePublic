package frc.robot.constants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class kSwerve {


    public static final double WHEEL_DIAMETER_METERS = 0.10033;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0
            * (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) // mk4 drive reduction
            * WHEEL_DIAMETER_METERS * Math.PI;
    public static final double MAX_ACCELERATION = 4; //1.7;
    public static double MAX_ANGULAR_ACCEL_RADIANS_PER_SECOND = 8.3;
    public static double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 13.96;


    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.445;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.445;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            // Front Right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front Left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back Right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back Left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 271.0;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 8.0;
    public static final double REAR_LEFT_MODULE_STEER_OFFSET = 54.0;
    public static final double REAR_RIGHT_MODULE_STEER_OFFSET = 321.0;

    public static final double SWERVE_ALLOWED_OFFSET = 1.0;
    public static final double SWERVE_CORRECTION_SPEED = 0.05 * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    public static final double DRIVE_GEAR_RATIO = 6.75; // 6.75:1
    public static final double ANGLE_GEAR_RATIO = 12.8; // 12.8:1

    public static final double DRIVE_MOTOR_KP = 0.1;
    public static final double DRIVE_MOTOR_KI = 0.0;
    public static final double DRIVE_MOTOR_KD = 0.0;
    public static final double DRIVE_MOTOR_KF = 0.0;

    public static final double DRIVE_OPEN_LOOP_RAMP = 0.25;
    public static final double DRIVE_CLOSED_LOOP_RAMP = 0.0;
    public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
    public static final double DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;

    public static final double ANGLE_MOTOR_KP = 0.3;
    public static final double ANGLE_MOTOR_KI = 0.0;
    public static final double ANGLE_MOTOR_KD = 1.0;
    public static final double ANGLE_MOTOR_KF = 0.0;

    public static final double ANGLE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final double ANGLE_PEAK_CURRENT_LIMIT = 60;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;

    // TODO: THEORETICAL GAINS
    public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward( // real
            0.2, // Voltage to break static friction
            2.25, // Volts per meter per second
            0.17 // Volts per meter per second squared
    );
    // Steer feed forward
    public static final SimpleMotorFeedforward STEER_FF = new SimpleMotorFeedforward( // real
            0.55, // Voltage to break static friction
            0.23, // Volts per radian per second
            0.0056 // Volts per radian per second squared
    );
}
