// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drives;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
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

import java.util.ArrayList;
import java.util.List;

import static frc.robot.constants.kSwerve.*;

public class Drives extends SubsystemBase {
    private boolean runDrive = true;

    private final SwerveModule[] modules;

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(kCANIDs.DRIVETRAIN_PIGEON_ID, kCANIDs.CANIVORE_NAME);

    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();

    private double lastPigeonRotation;
    private DoubleLogEntry pigeonLog;

    private final BasePigeonSimCollection gyroSim = pigeon.getSimCollection();

    public Drives() {
        pigeon.configFactoryDefault();
        pigeon.reset();

        SmartDashboard.putData("Field", field);

        modules = new SwerveModule[] {
                new SwerveModule(0, kCANIDs.CANIVORE_NAME, kCANIDs.FRONT_RIGHT_DRIVE, kCANIDs.FRONT_RIGHT_STEER, kCANIDs.FRONT_RIGHT_CANCODER, FRONT_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(1, kCANIDs.CANIVORE_NAME, kCANIDs.FRONT_LEFT_DRIVE, kCANIDs.FRONT_LEFT_STEER, kCANIDs.FRONT_LEFT_CANCODER, FRONT_LEFT_MODULE_STEER_OFFSET),
                new SwerveModule(2, kCANIDs.CANIVORE_NAME, kCANIDs.REAR_RIGHT_DRIVE, kCANIDs.REAR_RIGHT_STEER, kCANIDs.REAR_RIGHT_CANCODER, REAR_RIGHT_MODULE_STEER_OFFSET),
                new SwerveModule(3, kCANIDs.CANIVORE_NAME, kCANIDs.REAR_LEFT_DRIVE, kCANIDs.REAR_LEFT_STEER, kCANIDs.REAR_LEFT_CANCODER, REAR_LEFT_MODULE_STEER_OFFSET)
        };

        odometry = new SwerveDriveOdometry(KINEMATICS, new Rotation2d(), getRealPositions(), new Pose2d());

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
        tab.addString("Current Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "");

        if(RobotBase.isReal()) {
            DataLog log = Robot.getDataLog();
            pigeonLog = new DoubleLogEntry(log, "Drives/pigeonRot");
        }
    }

    /**
     * Run an auto path from the available PathPlanner paths.
     *
     * @param pathName Name of path in PathPlanner
     * @return Command that runs an autonomous path
     */
    public Command commandRunGeneratedPath(String pathName) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kSwerve.MAX_VELOCITY_METERS_PER_SECOND, kSwerve.MAX_ACCELERATION);
        return commandRunPath(path).withName("CommandRunPath/" + pathName);
    }

    /**
     * Generate a trajectory based on a list of path points
     */
    public Command commandDriveTrajectory(List<PathPoint> pathPoints) {
        List<PathPoint> withStart = new ArrayList<>();
        withStart.add(PathPoint.fromCurrentHolonomicState(getPose(), KINEMATICS.toChassisSpeeds(getRealStates())));
        withStart.addAll(pathPoints);
        PathPlannerTrajectory path = PathPlanner.generatePath(kSwerve.MAX_VELOCITY_METERS_PER_SECOND, kSwerve.MAX_ACCELERATION, false, withStart.get(0), withStart.get(1), withStart.subList(2, pathPoints.size()).toArray(new PathPoint[0]));
        return commandRunPath(path).withName("CommandDriveTrajectory");
    }

    public Command commandRunPath(PathPlannerTrajectory path) {
        PathPlannerTrajectory.PathPlannerState initialState = path.getInitialState();
        Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    field.getObject("traj").setTrajectory(path);
                    field.getObject("beginpos").setPose(startingPose);
                    field.getObject("endpos").setPose(path.getEndState().poseMeters);
                }),
                Commands.runOnce(() -> setOdometryPose(startingPose)),
                new MCQSwerveControllerCommand(
                        path,
                        this::getPose,
                        kAuto.X_PID_CONTROLLER,
                        kAuto.Y_PID_CONTROLLER,
                        kAuto.THETA_AUTO_PID,
                        this::updateModules,
                        this
                ),
                Commands.runOnce(() -> this.updateModules(new ChassisSpeeds()))
        );
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
    public SwerveModulePosition[] getRealPositions() {
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
        pigeon.reset();
    }

    public Pigeon2 getGyro() {
        return pigeon;
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
        odometry.resetPosition(pigeon.getRotation2d(), getRealPositions(), pose);
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
     * Update the modules with a new ChassisSpeeds
     *
     * @param speeds The ChassisSpeeds to drive at.
     */
    public void updateModules(ChassisSpeeds speeds) {
        updateModules(KINEMATICS.toSwerveModuleStates(speeds));
    }

    /**
     * Update the modules with a new set of SwerveModuleStates
     *
     * @param newStates The states to set the modules to.
     */
    public void updateModules(SwerveModuleState[] newStates) {
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

    @Override
    public void periodic() {
        if(RobotBase.isReal()) {
            if (pigeon.getRotation2d().getDegrees() != lastPigeonRotation)
                pigeonLog.append(pigeon.getRotation2d().getDegrees());
            lastPigeonRotation = pigeon.getRotation2d().getDegrees();
        }

        odometry.update(pigeon.getRotation2d(), getRealPositions());
        // odometry.addVisionMeasurement(visionMeasure.getVisionPose(odometry.getPoseMeters()), visionMeasure.getTimestamp());

        var pose = getPose();
        field.setRobotPose(pose);
        //var target = visionMeasure.getObject(0);
        /*
        if(target != null){
            double distance = pose.getTranslation().getDistance(target);
            SmartDashboard.putNumber("distanceToTarget", distance);
        }
         */
    }

    @Override
    public void simulationPeriodic() {
        for (SwerveModule module: modules) {
            module.simulationPeriodic();
        }

        double chassisOmega = KINEMATICS.toChassisSpeeds(getRealStates()).omegaRadiansPerSecond;
        chassisOmega = Math.toDegrees(chassisOmega);
        gyroSim.addHeading(chassisOmega*0.02);
    }
}