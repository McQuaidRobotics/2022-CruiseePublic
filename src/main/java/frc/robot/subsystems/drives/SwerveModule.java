package frc.robot.subsystems.drives;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.kSwerve;
import frc.robot.utils.SwerveUtil;
import frc.robot.utils.UnitUtil;

import static frc.robot.constants.kSwerve.kDriveFF;
import static frc.robot.constants.kSwerve.kSteerFF;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final WPI_TalonFX angleMotor;
    private final WPI_TalonFX driveMotor;
    private final WPI_CANCoder angleEncoder;
    private double lastAngle;

    private final TalonFXSimCollection driveMotorSim;
    private final FlywheelSim driveWheelSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                    kDriveFF.kv * kSwerve.WHEEL_CIRCUMFERENCE_METERS / (2*Math.PI),
                    kDriveFF.ka * kSwerve.WHEEL_CIRCUMFERENCE_METERS / (2*Math.PI)
            ),
            DCMotor.getFalcon500(1),
            kSwerve.DRIVE_GEAR_RATIO
    );
    private final TalonFXSimCollection steerMotorSim;
    private final FlywheelSim steeringSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(kSteerFF.kv, kSteerFF.ka),
            DCMotor.getFalcon500(1),
            kSwerve.DRIVE_GEAR_RATIO
    );
    private final CANCoderSimCollection steerEncoderSim;


    public SwerveModule(int moduleNumber, String canbus, int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(cancoderID, canbus);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new WPI_TalonFX(angleMotorID, canbus);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new WPI_TalonFX(driveMotorID, canbus);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();

        driveMotorSim = driveMotor.getSimCollection();
        steerMotorSim = angleMotor.getSimCollection();
        steerEncoderSim = angleEncoder.getSimCollection();
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
        double distance = UnitUtil.positionToMeters(driveMotor.getSelectedSensorPosition(), kSwerve.DRIVE_GEAR_RATIO, kSwerve.WHEEL_CIRCUMFERENCE_METERS);
        Rotation2d angle = Rotation2d.fromDegrees(UnitUtil.falconToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModulePosition(distance, angle);
    }



    public Rotation2d getFalconWheelRotation() {
        return Rotation2d.fromDegrees(UnitUtil.positionToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
    }

    public void simulationPeriodic() {
        // apply our commanded voltage to our simulated physics mechanisms
        double driveVoltage = driveMotorSim.getMotorOutputLeadVoltage();
        if(driveVoltage >= 0) driveVoltage = Math.max(0, driveVoltage-kSteerFF.ks);
        else driveVoltage = Math.min(0, driveVoltage+kSteerFF.ks);
        driveWheelSim.setInputVoltage(driveVoltage);

        double steerVoltage = steerMotorSim.getMotorOutputLeadVoltage();
        if(steerVoltage >= 0) steerVoltage = Math.max(0, steerVoltage-kSteerFF.ks);
        else steerVoltage = Math.min(0, steerVoltage+kSteerFF.ks);
        steeringSim.setInputVoltage(steerVoltage);

        driveWheelSim.update(0.02);
        steeringSim.update(0.02);

        // update our simulated devices with our simulated physics results
        double driveMotorVelocityNative = UnitUtil.rotationsToVelocity(driveWheelSim.getAngularVelocityRPM()/60, kSwerve.DRIVE_GEAR_RATIO);
        double driveMotorPositionDeltaNative = driveMotorVelocityNative*10*0.02;
        driveMotorSim.setIntegratedSensorVelocity((int)driveMotorVelocityNative);
        driveMotorSim.addIntegratedSensorPosition((int)(driveMotorPositionDeltaNative));
        driveMotorSim.setSupplyCurrent(driveWheelSim.getCurrentDrawAmps()/2);

        double steerMotorVelocityNative = UnitUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, kSwerve.ANGLE_GEAR_RATIO);
        double steerMotorPositionDeltaNative = steerMotorVelocityNative*10*0.02;
        steerMotorSim.setIntegratedSensorVelocity((int)steerMotorVelocityNative);
        steerMotorSim.addIntegratedSensorPosition((int)(steerMotorPositionDeltaNative));
        steerMotorSim.setSupplyCurrent(steeringSim.getCurrentDrawAmps()/2);

        steerEncoderSim.setVelocity((int)(UnitUtil.rotationsToVelocity(steeringSim.getAngularVelocityRPM()/60, 1)*2));
        steerEncoderSim.setRawPosition((int)(getFalconWheelRotation().getDegrees()/360.0*4096));

        driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        steerEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("driveSim/driveVoltage", driveVoltage);
        SmartDashboard.putNumber("driveSim/steerVoltage", steerVoltage);
        SmartDashboard.putNumber("driveSim/driveRPM", driveWheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("driveSim/steerRPM", steeringSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("driveSim/busVoltage", RobotController.getBatteryVoltage());
    }

    public double getDriveCurrentDraw(){
        return driveMotor.getSupplyCurrent();
    }
    public double getSteerCurrentDraw(){
        return angleMotor.getSupplyCurrent();
    }
}