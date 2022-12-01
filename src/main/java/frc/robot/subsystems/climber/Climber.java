// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kClimb;
import frc.robot.constants.kPneumatics;

public class Climber extends SubsystemBase {
    public ClimberArm outerArm, innerArm;

    private final CANSparkMax sidewaysMover;

    private final DoubleSolenoid lock = new DoubleSolenoid(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, kPneumatics.CLIMB_BREAK_2, kPneumatics.CLIMB_BREAK_1);

    public Climber() {
        innerArm = new ClimberArm(kCANIDs.INNER_ANGLE,kCANIDs.INNER_REACH, 
                    kClimb.climbAngleInner, kClimb.climbReachInner, false);
        outerArm = new ClimberArm(kCANIDs.OUTER_ANGLE,kCANIDs.OUTER_REACH, 
                    kClimb.climbAngleOuter, kClimb.climbReachOuter, true);
        sidewaysMover = new CANSparkMax(kCANIDs.SIDEWAYS_MOVER, MotorType.kBrushless);
    }

    public Command commandRunSidewaysMover(double speed) {
        return Commands.startEnd(
                () -> moveSidewaysPOut(speed),
                () -> moveSidewaysPOut(0),
                this
        );
    }

    public Command commandReleaseLock(){
        return Commands.runOnce(this::releaseLock, this).withName("ReleaseClimbLock");
    }

    public Command commandExtendLock(){
        return Commands.runOnce(this::extendLock, this).withName("ExtendClimbLock");
    }

    public void setToCoast(){
        outerArm.setAngleToCoast();
        innerArm.setAngleToCoast();
        outerArm.setReachToCoast();
        innerArm.setReachToCoast();
    }

    public void setToBrake(){
        outerArm.setAngleToBrake();
        innerArm.setAngleToBrake();
        outerArm.setReachToBrake();
        innerArm.setReachToBrake();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder Out Reach", outerArm.reachEncoder.getPosition());
        SmartDashboard.putNumber("Climb Encoder Out Angle", outerArm.angleEncoder.getPosition()*kClimb.CLIMB_ROTATION_TO_DEGREE);
        SmartDashboard.putNumber("Climb Encoder In Reach", innerArm.reachEncoder.getPosition());
        SmartDashboard.putNumber("Climb Encoder In Angle", innerArm.angleEncoder.getPosition()*kClimb.CLIMB_ROTATION_TO_DEGREE);
        SmartDashboard.putNumber("Climb Setpoint Out Reach", outerArm.getReachSetpoint());
        SmartDashboard.putNumber("Climb Setpoint In Reach", innerArm.getReachSetpoint());
        SmartDashboard.putBoolean("Climb lock", lock.get() == Value.kForward);

        SmartDashboard.putNumber("Sideways Mover Current", sidewaysMover.getOutputCurrent());

        outerArm.periodic();
        innerArm.periodic();
    }

    public void zeroAngleEncoders(){
        innerArm.zeroAngleEncoders();
        outerArm.zeroAngleEncoders();
    }

    public void startInitialize(){
        innerArm.startInitialize();
        outerArm.startInitialize();
        releaseLock();
    }

    public void endInitialize(){
        innerArm.endInitialize();
        outerArm.endInitialize();
    }

    public void releaseLock(){
        lock.set(Value.kReverse);
    }

    public void extendLock(){
        lock.set(Value.kForward);
    }

    public void moveSidewaysPOut(double percent){
        sidewaysMover.set(percent);
    }
}
