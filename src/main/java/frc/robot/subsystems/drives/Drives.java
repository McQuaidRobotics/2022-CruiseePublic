// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drives;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kAuto;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kSwerve;
import frc.robot.utils.MCQSwerveControllerCommand;
import frc.robot.utils.PoseCamera;

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
    private final SwerveDriveOdometry odometry;
    // private final SwerveDrivePoseEstimator<N7, N7, N7> odometry;
    private final Field2d field = new Field2d();

    private double lastPigeonRotation;
    private DoubleLogEntry pigeonLog;
    private final PoseCamera visionMeasure = new PoseCamera("gloworm");

    public Drives() {
        pigeonTwo.configFactoryDefault();
        pigeonTwo.reset();

        SmartDashboard.putData("Field", field);

        modules = new SwerveModule[] {
                new SwerveModule(0, CANIVORE_NAME, kCANIDs.FRONT_RIGHT_DRIVE, kCANIDs.FRONT_RIGHT_STEER, kCANIDs.FRONT_RIGHT_CANCODER, FRONT_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(1, CANIVORE_NAME, kCANIDs.FRONT_LEFT_DRIVE, kCANIDs.FRONT_LEFT_STEER, kCANIDs.FRONT_LEFT_CANCODER, FRONT_LEFT_MODULE_STEER_OFFSET),
                new SwerveModule(2, CANIVORE_NAME, kCANIDs.REAR_RIGHT_DRIVE, kCANIDs.REAR_RIGHT_STEER, kCANIDs.REAR_RIGHT_CANCODER, REAR_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(3, CANIVORE_NAME, kCANIDs.REAR_LEFT_DRIVE, kCANIDs.REAR_LEFT_STEER, kCANIDs.REAR_LEFT_CANCODER, REAR_LEFT_MODULE_STEER_OFFSET)
        };


        // odometry = new SwerveDrivePoseEstimator<>(Nat.N7(),Nat.N7(),Nat.N7(),
        //                          new Rotation2d(0),
        //                         new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        //                         getModulePositions(),
        //                         kinematics,
        //                         new MatBuilder<>(Nat.N7(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.001,0.001,0.001,0.001),
        //                         new MatBuilder<>(Nat.N7(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.001,0.001,0.001,0.001),
        //                         new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        //                 );
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0), getRealPositions(), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

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

        if(RobotBase.isReal()) {
            DataLog log = Robot.getDataLog();
            pigeonLog = new DoubleLogEntry(log, "Drives/pigeonRot");
        }

        visionMeasure.addVisionTargetPose(0.5, 0.5);
    }

    /**
     * Run an auto path from the available PathPlanner paths.
     *
     * @param pathName Name of path in PathPlanner
     * @return Command that runs an autonomous path
     */
    public Command runAutoPath(String pathName) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kSwerve.MAX_VELOCITY_METERS_PER_SECOND, kSwerve.MAX_ACCELERATION);

        PathPlannerTrajectory.PathPlannerState initialState = path.getInitialState();
        Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

        return Commands.sequence(
                Commands.run(() -> {
                    field.getObject("traj").setTrajectory(path);
                    field.getObject("beginpos").setPose(path.getInitialState().poseMeters);
                    field.getObject("endpos").setPose(path.getEndState().poseMeters);
                }),
                Commands.run(() -> setOdometryPose(startingPose)),
                new MCQSwerveControllerCommand(
                        path,
                        this::getPose,
                        getKinematics(),
                        kAuto.X_PID_CONTROLLER,
                        kAuto.Y_PID_CONTROLLER,
                        kAuto.THETA_AUTO_PID,
                        this::updateModules,
                        this
                )
        ).withName("AutonomousCommand/" + pathName);
    }

    /**
     * Zero the gyro.
     *
     * @return Command that zeros the gyroscope
     */
    public Command commandZeroGyroscope() {
        return Commands.runOnce(this::zeroGyroscope, this).withName("ZeroGyroscope");
    }

    /**
     * Gets the current positions of swerve modules.
     *
     * @return Array of swerve module positions.
     */
    public SwerveModulePosition[] getRealPositions(){
       SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    /**
     * Gets the current states of the modules (vectors).
     *
     * @return Array of swerve module states
     */
    public SwerveModuleState[] getRealStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
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
        return odometry.getPoseMeters().getRotation();
    }

    /**
     * Sets the current pose of the odometry.
     *
     * @param pose New pose.
     */
    public void setOdometryPose(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), getRealPositions(), getPose());
    }

    /**
     * Get the current odometry pose
     *
     * @return Current odometry pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Update the modules with a new set of SwerveModuleStates
     *
     * @param newStates The states to set the modules to.
     */
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

    @Override
    public void periodic() {
        if(RobotBase.isReal()) {
            if (pigeonTwo.getRotation2d().getDegrees() != lastPigeonRotation)
                pigeonLog.append(pigeonTwo.getRotation2d().getDegrees());
            lastPigeonRotation = pigeonTwo.getRotation2d().getDegrees();
        }

        odometry.update(pigeonTwo.getRotation2d(), getRealPositions());
        // odometry.addVisionMeasurement(visionMeasure.getVisionPose(odometry.getPoseMeters()), visionMeasure.getTimestamp());
        
        SmartDashboard.putNumber("odometry.getEstimatedPositionX", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometry.getEstimatedPositionY", odometry.getPoseMeters().getY());
        var pose = getPose();
        field.setRobotPose(pose);
        var target = visionMeasure.getObject(0);
        /*
        if(target != null){
            double distance = pose.getTranslation().getDistance(target);
            SmartDashboard.putNumber("distanceToTarget", distance);
        }
         */
    }
}