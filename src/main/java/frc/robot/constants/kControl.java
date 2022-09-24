// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.subsystems.Shooter.ShooterRPMS;

/** Add your docs here. */
public class kControl {
    // Index
    public static final double INDEX_ALLOWED_ERROR_ROTATIONS = 0.5;
    public static final double INDEX_ONE_BALL_ROTATIONS = 80; // Guess for shooter
    public static final double INDEX_MOVE_BACK = -10;

    // Acquisition
    public static final double ACQUISITION_RPMS = 5000;

    // Shooter
    public static final ShooterRPMS SHOOTER_LOW_RPMS = new ShooterRPMS(500, 4000);
    public static final ShooterRPMS SHOOTER_HIGH_RPMS = new ShooterRPMS(4500, 5000);
    public static final ShooterRPMS SHOOTER_AUTO_RPMS = new ShooterRPMS(4400, 4900);
}
