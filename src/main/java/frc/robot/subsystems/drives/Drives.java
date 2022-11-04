// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drives;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.constants.kVision;
import frc.robot.utils.PoseCamera;
import edu.wpi.first.math.numbers.N1;

import static frc.robot.constants.kSwerve.*;

public class Drives extends SubsystemBase {
    private boolean runDrive = true;

    private final SwerveModule[] modules;

    private final WPI_Pigeon2 pigeonTwo = new WPI_Pigeon2(kCANIDs.DRIVETRAIN_PIGEON_ID, kSwerve.CANIVORE_NAME);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                    // Front Right
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Front Left
                    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Right
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                    // Back Left
                    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    private final SwerveDrivePoseEstimator odometry;
    private final Field2d field = new Field2d();

    private final DoubleLogEntry pigeonLog;
    private final PoseCamera visionMeasure = new PoseCamera("gloworm");

    public Drives() {
        pigeonTwo.configFactoryDefault();
        pigeonTwo.reset();
        
        odometry = new SwerveDrivePoseEstimator( new Rotation2d(0), 
                                new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
                                kinematics,
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
                                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), 
                                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01) 
                        );

        SmartDashboard.putData("Field", field);

        modules = new SwerveModule[] {
                new SwerveModule(0, CANIVORE_NAME, kCANIDs.FRONT_RIGHT_DRIVE, kCANIDs.FRONT_RIGHT_STEER, kCANIDs.FRONT_RIGHT_CANCODER, FRONT_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(1, CANIVORE_NAME, kCANIDs.FRONT_LEFT_DRIVE, kCANIDs.FRONT_LEFT_STEER, kCANIDs.FRONT_LEFT_CANCODER, FRONT_LEFT_MODULE_STEER_OFFSET),
                new SwerveModule(2, CANIVORE_NAME, kCANIDs.REAR_RIGHT_DRIVE, kCANIDs.REAR_RIGHT_STEER, kCANIDs.REAR_RIGHT_CANCODER, REAR_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(3, CANIVORE_NAME, kCANIDs.REAR_LEFT_DRIVE, kCANIDs.REAR_LEFT_STEER, kCANIDs.REAR_LEFT_CANCODER, REAR_LEFT_MODULE_STEER_OFFSET)
        };

        ShuffleboardTab tab = Shuffleboard.getTab("Drives");

        tab.add(field).withSize(4, 4).withPosition(0, 0);
        for (SwerveModule module : modules) {
            ShuffleboardLayout moduleLayout = tab.getLayout("Module " + module.moduleNumber, BuiltInLayouts.kList)
                    .withPosition(5 + module.moduleNumber * 2, 0)
                    .withSize(2, 3);
            moduleLayout.addNumber("Absolute Rotation", () -> module.getWheelRotation().getDegrees());
            moduleLayout.addNumber("Falcon Rotation", () -> module.getState().angle.getDegrees());
            moduleLayout.addNumber("Speed MPS", () -> module.getState().speedMetersPerSecond);
        }

        DataLog log = Robot.getDataLog();
        pigeonLog = new DoubleLogEntry(log, "Drives/pigeonRot");

        visionMeasure.addVisionTargetPose(0.5, 0.5);
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeonTwo.reset();
    }

    public Pigeon2 getGyro() {
        return pigeonTwo;
    }

    public Rotation2d getRotation() {
        return odometry.getEstimatedPosition().getRotation();
    }

    public void setOdometryPose(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public void updateModules(SwerveModuleState[] newStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MAX_VELOCITY_METERS_PER_SECOND);

        for(SwerveModule module : modules) {
            module.setDesiredState(newStates[module.moduleNumber]);
        }
    }

    public void setRunDrives(boolean runDrive){
        this.runDrive = runDrive;
    }

    public boolean getRunDrives(){
        return runDrive;
    }

    public Field2d getField() {
        return field;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModuleState[] getRealStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    @Override
    public void periodic() {
        pigeonLog.append(pigeonTwo.getRotation2d().getDegrees());

        odometry.update(pigeonTwo.getRotation2d(), getRealStates());
        odometry.addVisionMeasurement(visionMeasure.getVisionPose(odometry.getEstimatedPosition()), visionMeasure.getTimestamp());
        
        SmartDashboard.putNumber("odometry.getEstimatedPositionX", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odometry.getEstimatedPositionY", odometry.getEstimatedPosition().getY());
        var pose = getPose();
        field.setRobotPose(pose);
        var target = visionMeasure.getObject(0);
        if(target != null){
            double distance = pose.getTranslation().getDistance(target);
            // SmartDashboard.putNumber("distanceToTarget", distance);
        }
    }
}