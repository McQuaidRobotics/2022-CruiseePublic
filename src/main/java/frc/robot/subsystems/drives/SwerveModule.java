package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.kSwerve;
import frc.robot.utils.SwerveUtil;
import frc.robot.utils.UnitUtil;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final TalonFX angleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;
    private double lastAngle;


    public SwerveModule(int moduleNumber, String canbus, int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(cancoderID, canbus);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorID, canbus);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID, canbus);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveUtil.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);

        if(percentOutput != 0) {
            double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
            angleMotor.set(ControlMode.Position, UnitUtil.degreesToFalcon(angle, kSwerve.ANGLE_GEAR_RATIO));
            lastAngle = angle;
        }
    }

    private void resetToAbsolute() {
        double absolutePosition = UnitUtil.degreesToFalcon(getWheelRotation().getDegrees() - angleOffset, kSwerve.ANGLE_GEAR_RATIO);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();

        var config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = false;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.magnetOffsetDegrees = 0;
        angleEncoder.configAllSettings(config);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();

        var config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                kSwerve.ANGLE_CONTINUOUS_CURRENT_LIMIT,
                kSwerve.ANGLE_PEAK_CURRENT_LIMIT,
                kSwerve.ANGLE_PEAK_CURRENT_DURATION
        );
        config.slot0.kP = kSwerve.ANGLE_MOTOR_KP;
        config.slot0.kI = kSwerve.ANGLE_MOTOR_KI;
        config.slot0.kD = kSwerve.ANGLE_MOTOR_KD;
        config.slot0.kF = kSwerve.ANGLE_MOTOR_KF;
        config.supplyCurrLimit = angleSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        angleMotor.configAllSettings(config);

        angleMotor.setInverted(false);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();

        var config = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                kSwerve.DRIVE_CONTINUOUS_CURRENT_LIMIT,
                kSwerve.DRIVE_PEAK_CURRENT_LIMIT,
                kSwerve.DRIVE_PEAK_CURRENT_DURATION
        );
        config.slot0.kP = kSwerve.DRIVE_MOTOR_KP;
        config.slot0.kI = kSwerve.DRIVE_MOTOR_KI;
        config.slot0.kD = kSwerve.DRIVE_MOTOR_KD;
        config.slot0.kF = kSwerve.DRIVE_MOTOR_KF;
        config.supplyCurrLimit = driveSupplyLimit;
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        config.openloopRamp = kSwerve.DRIVE_OPEN_LOOP_RAMP;
        config.closedloopRamp = kSwerve.DRIVE_CLOSED_LOOP_RAMP;
        driveMotor.configAllSettings(config);

        driveMotor.setInverted(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getWheelRotation() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = UnitUtil.falconToMPS(driveMotor.getSelectedSensorVelocity(), kSwerve.WHEEL_CIRCUMFERENCE_METERS, kSwerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(UnitUtil.falconToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = UnitUtil.falconToM(driveMotor.getSelectedSensorPosition(), kSwerve.WHEEL_CIRCUMFERENCE_METERS, kSwerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(UnitUtil.falconToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModulePosition(distance, angle);
    }
}