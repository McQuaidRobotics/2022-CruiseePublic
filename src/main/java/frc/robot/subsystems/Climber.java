// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ClimbConstants;
import frc.robot.utils.ClimberArm;
import frc.robot.constants.kCANIDs;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    public ClimberArm outerArm;
    public ClimberArm innerArm;
    public Climber() {
        innerArm = new ClimberArm(kCANIDs.INNER_ANGLE,kCANIDs.INNER_REACH, 
                    ClimbConstants.climbAngleInner, ClimbConstants.climbReachInner, false);
        outerArm = new ClimberArm(kCANIDs.OUTER_ANGLE,kCANIDs.OUTER_REACH, 
                    ClimbConstants.climbAngleOuter, ClimbConstants.climbReachOuter, true);
        
    }

    public void extendArm(ClimberArm arm, double distance){
        double rotations = distance/ClimbConstants.CLIMB_ROTATION_TO_INCH;
        SmartDashboard.putNumber("Climb Setpoint Reach", rotations);
        arm.setReachSetpoint(rotations);
    }  
    public void rotateArmTo(ClimberArm arm, double angle){
        double rotations = angle/ClimbConstants.CLIMB_ROTATION_TO_DEGREE;
        SmartDashboard.putNumber("Climb Setpoint Angle", rotations);
        arm.setAngleSetpoint(rotations);
    }  
    public void setToCoast(){
        outerArm.setAngleToCoast();
        innerArm.setAngleToCoast();
        outerArm.setReachToCoast();
        innerArm.setReachToCoast();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder Reach", outerArm.reachEncoder.getPosition());
        SmartDashboard.putNumber("Climb Encoder Angle", outerArm.angleEncoder.getPosition());
        // This method will be called once per scheduler run
    }
}
