// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.kAuto;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.drives.Drives;

/** Add your docs here. */
public class AutoUtil {
    public enum Routine {
        HANGAR_TWO_BALL,
        TERMINAL_TWO_BALL,
        POTATO,
        NOTHING,
        DEFAULT,
        TEST
    }

    public static Command generateCommand(String pathName, Drives drives) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, kSwerve.MAX_VELOCITY_METERS_PER_SECOND, kSwerve.MAX_ACCELERATION);

        PathPlannerTrajectory.PathPlannerState initialState = path.getInitialState();
        Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

        return new SequentialCommandGroup(
                new InstantCommand(() -> logPath(path, drives.getField())),
                new InstantCommand(() -> drives.setOdometryPose(startingPose)),
                new MCQSwerveControllerCommand(
                    path,
                    drives::getPose,
                    drives.getKinematics(),
                    kAuto.X_PID_CONTROLLER,
                    kAuto.Y_PID_CONTROLLER,
                    kAuto.THETA_PID_CONTROLLER,
                    drives::updateModules
                )
        );
    }

    private static void logPath(PathPlannerTrajectory path, Field2d field) {
        field.getObject("traj").setTrajectory(path);
        field.getObject("beginpos").setPose(path.getInitialState().poseMeters);
        field.getObject("endpos").setPose(path.getEndState().poseMeters);
    }
}
