// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import frc.robot.constants.kVision;

/** Add your docs here. */
public class PoseCamera {
    PhotonVisionWrapper camera;
    ArrayList<Translation2d> objects;
    Debouncer targetDebouncer = new Debouncer(kVision.DEBOUNCE_TIME, DebounceType.kRising);
    
    public PoseCamera(String name){
        camera = new PhotonVisionWrapper(name);
        objects = new ArrayList<Translation2d>();
    }
    public double getDistance(){return camera.getDistance(); }
    public double getTimestamp(){ return camera.getTimestamp(); }
    public void addVisionTargetPose(double x, double y){
        objects.add(new Translation2d(kVision.FIELD_WIDTH*x, kVision.FIELD_HEIGHT*y));
    }
    public Translation2d getObject(int idx){
        if(idx < objects.size()){
            return objects.get(idx);
        }
        return null;
    }
    public Pose2d getVisionPose(Pose2d current){
        double distance = getDistance();

        if(distance == -1 || !targetDebouncer.calculate(distance != -1)){
            return current;
        }
        Rotation2d theta = current.getRotation();
        Translation2d absPose;
        double bestDistance = Integer.MAX_VALUE;
        double x = distance*theta.getCos();
        double y = distance*theta.getSin();

        Translation2d bestPose = new Translation2d();
        for(Translation2d target : objects){
            absPose =  new Translation2d(target.getX() - x, target.getY() - y);
            if(absPose.getDistance(current.getTranslation()) < bestDistance){
                bestDistance = absPose.getDistance(current.getTranslation()); 
                bestPose = absPose;
            }
        }
        SmartDashboard.putNumber("BestPoseX", bestPose.getX());
        SmartDashboard.putNumber("BestPoseY", bestPose.getY());
        return new Pose2d(bestPose, theta);
    }
}
