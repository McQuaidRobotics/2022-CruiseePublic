// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.kAuto;
import frc.robot.subsystems.Drives;

/** Add your docs here. */
public class AutoUtil {
    public enum Routine {
        FOUR_BALL,
        THREE_BALL,
        TWO_BALL,
        POTATO
    }

    public static Command generateCommand(String pathName, double maxVelocity, double maxAcceleration, Drives drives) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, maxVelocity, maxAcceleration);

        return new SequentialCommandGroup(
                new InstantCommand(() -> logPath(path, drives.getField())),
                new InstantCommand(() -> drives.setOdometryRotation(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))),
                new PPSwerveControllerCommandWrapper(
                    path,
                    drives::getPose,
                    drives.kinematics,
                    kAuto.LEFT_PID_CONTROLLER,
                    kAuto.RIGHT_PID_CONTROLLER,
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
