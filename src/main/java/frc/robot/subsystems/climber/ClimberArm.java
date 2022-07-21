// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.kClimb;
import frc.robot.constants.kClimb.ClimberPid;

/** Add your docs here. */
public class ClimberArm {
    private final CANSparkMax angleMotor, reachMotor;

    public RelativeEncoder angleEncoder, reachEncoder;

    private final SparkMaxPIDController anglePidController, reachPidController;

    private final ClimberPid anglePid, reachPid;

    private double reachSetpoint = 0;
    private double angleSetpoint = 0;

    private final boolean isReversedReach;

    private final Debouncer debouncerAngle = new Debouncer(0.1, DebounceType.kBoth);

    public ClimberArm(int angleId, int reachId, ClimberPid anglePID, ClimberPid reachPID, boolean isReversedReach){
        angleMotor = new CANSparkMax(angleId, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(false);
        angleMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setClosedLoopRampRate(2);

        reachMotor = new CANSparkMax(reachId, MotorType.kBrushless);
        reachMotor.restoreFactoryDefaults();
        reachMotor.setInverted(isReversedReach);
        reachMotor.setIdleMode(IdleMode.kBrake);
        reachMotor.setClosedLoopRampRate(0.25);

        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPosition(0);
        
        reachEncoder = reachMotor.getEncoder();
        reachEncoder.setPosition(0);
        
        anglePidController = angleMotor.getPIDController();
        reachPidController = reachMotor.getPIDController();
        kClimb.addPidToMotor(anglePidController, anglePID);
        kClimb.addPidToMotor(reachPidController, reachPID);

        reachPid = reachPID;
        anglePid = anglePID;

        // Set limits for reach
        reachMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(kClimb.CLIMB_MIN_EXTEND/kClimb.CLIMB_ROTATION_TO_INCH));
        reachMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        reachMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(kClimb.CLIMB_MAX_EXTEND/kClimb.CLIMB_ROTATION_TO_INCH));
        reachMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        
        // Set soft limits for angle
        angleMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(-26/kClimb.CLIMB_ROTATION_TO_DEGREE));
        angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        angleMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(45/kClimb.CLIMB_ROTATION_TO_DEGREE));
        angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

        setReachSetpoint(kClimb.CLIMB_MIN_EXTEND/kClimb.CLIMB_ROTATION_TO_INCH);

        this.isReversedReach = isReversedReach;
    }

    public void setReachSetpoint(double rotations){
        reachPidController.setReference(rotations, ControlType.kPosition);
        reachSetpoint = rotations;
    }

    public void setAngleSetpoint(double rotations){
        anglePidController.setReference(rotations, ControlType.kPosition);
        angleSetpoint = rotations;
    }

    public void periodic(){
        SmartDashboard.putNumber("A-AngClm" + (isReversedReach?"O":"I"), angleMotor.getOutputCurrent());
        SmartDashboard.putNumber("A-RchClm" + (isReversedReach?"O":"I"), reachMotor.getOutputCurrent());
    }

    public void setAngleToCoast(){
        angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setReachToCoast(){
        reachMotor.setIdleMode(IdleMode.kCoast);
    }

    public void setReachToBrake(){
        reachMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setAngleToBrake(){
        angleMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getReachSetpoint(){
        return reachSetpoint;
    }

    public void moveAnglePOut(double percent){
        angleMotor.set(percent);
    }

    public void moveReachPOut(double percent){
        reachMotor.set(percent);
    }

    public double getReachCurrent(){
        return reachMotor.getOutputCurrent();
    }

    public double getAngleCurrent(){
        return angleMotor.getOutputCurrent();
    }

    public double calculateReachError(){
        return Math.abs(reachSetpoint - reachEncoder.getPosition());
    }

    public double calculateAngleError(){
        return Math.abs(angleSetpoint - angleEncoder.getPosition());
    }

    public void disableAngleSoftLimits(){
        angleMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void disableReachSoftLimits(){
        reachMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        reachMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void enableAngleSoftLimits(){
        angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void enableReachSoftLimits(){
        reachMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        reachMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void zeroAngleEncoders(){    
        angleEncoder.setPosition(0);
    }

    public void zeroReachEncoders(){    
        reachEncoder.setPosition(0);    
    }

    public void startInitialize(){
        disableAngleSoftLimits();
        disableReachSoftLimits();

        anglePidController.setOutputRange(-0.5, 0.5);
        angleMotor.setClosedLoopRampRate(2);
        
        reachPidController.setOutputRange(-0.35, 0.35);
        reachMotor.setClosedLoopRampRate(0.25);
    }

    public void endInitialize(){
        zeroAngleEncoders();
        zeroReachEncoders();

        enableAngleSoftLimits();
        enableReachSoftLimits();

        anglePidController.setOutputRange(anglePid.min, anglePid.max);
        angleMotor.setClosedLoopRampRate(2);
        
        reachPidController.setOutputRange(reachPid.min, reachPid.max);
        reachMotor.setClosedLoopRampRate(0.25);
    }

    public void setAngleSmartLimit(int limit){
        angleMotor.setSmartCurrentLimit(limit);
    }

    public boolean debounceCurrentAngle(boolean isHigh){
        return debouncerAngle.calculate(isHigh);
    }

    public double getAngle(){
        return angleEncoder.getPosition()*kClimb.CLIMB_ROTATION_TO_DEGREE;
    }

    public void setReachOutput(double min, double max){
        reachPidController.setOutputRange(min, max);
    }
}
