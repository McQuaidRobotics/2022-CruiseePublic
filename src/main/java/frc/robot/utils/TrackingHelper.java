// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.drives.Drives;

/** Add your docs here. */
public class TrackingHelper {
    final double FORWARD_ANGLE = 7;
    final double CHANGE_ANGLE = 0.25;
    final double FORWARD_SPEED = 0.7;
    final double DEGREE_RANGE = 25;
    Drives drives;
    PhotonVisionWrapper cam;
    Rotation2d driveRotation = new Rotation2d();
    Rotation2d ballRotation = new Rotation2d();
    Rotation2d lastBallRot = new Rotation2d();
    double distance = 0;
    Rotation2d targetRotation = new Rotation2d();
    Debouncer detectionDebounce = new Debouncer(1, DebounceType.kFalling);
    public TrackingHelper(Drives drives){
        this.drives = drives;
        this.cam = new PhotonVisionWrapper("gloworm");
    }
    public void update(){
        driveRotation = drives.getRotation();
        distance = cam.getDistance();
        double rawBall = cam.getYaw();
        ballRotation = Rotation2d.fromDegrees(rawBall);
        targetRotation = driveRotation.plus(ballRotation);
        SmartDashboard.putNumber("targetRotation", targetRotation.getDegrees());
        
        lastBallRot = ballRotation;
    }
    public double getRotSpeed(){
        SmartDashboard.putNumber("raw rotSpeed", (
            targetRotation.minus(driveRotation)).getDegrees());

        double rotSpeed = (((targetRotation.minus(driveRotation)).getDegrees()/DEGREE_RANGE) * -0.1)*kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND ; 
        
        return rotSpeed;
    }
    public boolean isFinished(){
        return !detectionDebounce.calculate(cam.hasTargets());
    }
    public boolean hasTarget(){
        SmartDashboard.putBoolean("hasTarget", cam.hasTargets());
        return cam.hasTargets();
    }
}
