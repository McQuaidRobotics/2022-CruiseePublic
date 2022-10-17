// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.kAuto;
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
    double distance = 0;
    Rotation2d targetRotation = new Rotation2d();
    Debouncer detectionDebounce = new Debouncer(1, DebounceType.kFalling);
    public static class RollingAverage{
        double[] vals;
        int length = 0;
        int idx = 0;
        
        public RollingAverage(int size){
            vals = new double[size];
            for(int i = 0; i < vals.length; i++){
                vals[i] = 0;
            }
        }
        public void add(double val){
            vals[idx] = val;
            idx = (idx+1)%vals.length;
            length = Math.min(length+1, vals.length);
        }
        public double get(){
            double total = 0;
            for(int i = 0; i < length; i++){
                total += vals[i];
            }
            return total / length;
        }
    }
    RollingAverage angleFilter = new RollingAverage(10); 
    

    // KalmanFilter<N2, N1, N1> rotationFilter = new KalmanFilter<N2, N1, N1>(Nat.N2(), Nat.N1(), 
    //                                             kSwerve.DRIVE_CONTROL_PLANT, 
    //                                             VecBuilder.fill(0.1),
    //                                             VecBuilder.fill(0.5),
    //                                             0.020);
    

    
    public TrackingHelper(Drives drives){
        this.drives = drives;
        this.cam = new PhotonVisionWrapper("gloworm");
    }
    public void update(){
        if(cam.hasTargets()){
            driveRotation = drives.getRotation();
            double rawBall = cam.getYaw();
            angleFilter.add(rawBall);
            ballRotation = Rotation2d.fromDegrees(angleFilter.get());
            targetRotation = driveRotation.minus(ballRotation);
            SmartDashboard.putNumber("targetRotation", targetRotation.getDegrees());
        }
    }
    
    public double getRotSpeed(){
        SmartDashboard.putNumber("raw rotSpeed", (
            targetRotation.minus(driveRotation)).getDegrees());

        double rotSpeed = kAuto.THETA_AIMING_PID.calculate(driveRotation.getRadians(), targetRotation.getRadians());
        
        //(((targetRotation.minus(driveRotation)).getDegrees()/DEGREE_RANGE) * -0.1)*kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND ; 
        
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
