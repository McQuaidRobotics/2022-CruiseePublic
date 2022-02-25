// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase
        public static final int PDP_ID = 61;

        public static final int DRIVETRAIN_PIGEON_ID = 33; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 22; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(221.0); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(8.0); // FIXME Measure and set front right steer offset

        public static final int REAR_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
        public static final int REAR_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set back left steer motor ID
        public static final int REAR_LEFT_MODULE_STEER_ENCODER = 23; // FIXME Set back left steer encoder ID
        public static final double REAR_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(54.0); // FIXME Measure and set back left steer offset

        public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set back right drive motor ID
        public static final int REAR_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set back right steer motor ID
        public static final int REAR_RIGHT_MODULE_STEER_ENCODER = 24; // FIXME Set back right steer encoder ID
        public static final double REAR_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(322.0); // FIXME Measure and set back right steer offset
        
        public static final double SWERVE_ALLOWED_OFFSET = 1.0;
        public static double SWERVE_CORRECTION_SPEED = 0.05 * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
}
