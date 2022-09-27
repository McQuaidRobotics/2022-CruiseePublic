// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kDIO;


public class Index extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;
  private final DigitalInput beambreak;

  private double lastIndexAmps;
  private final DoubleLogEntry indexAmperageLog;
  private IndexState indexState;

  public Index() {
    motor = new CANSparkMax(kCANIDs.IDX_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    encoder = motor.getEncoder();
    pidController = motor.getPIDController();
    pidController.setP(0.3);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setOutputRange(-1, 1);

    beambreak = new DigitalInput(kDIO.BEAMBREAK);

    ShuffleboardTab tab = Shuffleboard.getTab("Index");
    tab.addNumber("Balls", this::getBallsIndexed).withPosition(0, 0).withSize(1, 1);
    tab.addBoolean("Beambreak", beambreak::get).withPosition(1, 0).withSize(1, 1);
    tab.addNumber("Index Position", this::getIndexPosition).withPosition(2, 0).withSize(1, 1);

    DataLog log = Robot.getDataLog();
    indexAmperageLog = new DoubleLogEntry(log, "Idx/Index-Amps");
    indexState = new IndexState(BallState.NONE);
  }

  @Override
  public void periodic() {
    if(motor.getOutputCurrent() != lastIndexAmps) indexAmperageLog.append(motor.getOutputCurrent());
    lastIndexAmps = motor.getOutputCurrent();
    SmartDashboard.putNumber("Index Position", getIndexPosition());
  }

  public boolean isBallBlockingBeam(){
    return !beambreak.get();
  }

  public double getIndexPosition(){
    return encoder.getPosition();
  }

  public int getBallsIndexed(){
    return indexState.getNumber();
  }
  public BallState getState(){
    return indexState.getState();
  }
  public BallState getDesiredState(){
    return indexState.getDesiredState();
  }
  public boolean wantsDifferentState(){
    return getDesiredState() != getState();
  }


  public void runClosedLoopPosition(double rotations){
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }
  public void runPercentOut(double percent){
    motor.set(percent);
  }

  public void update(boolean newDetected){
    indexState.update(newDetected);
  }

  public void removeBall(){
    indexState.removeBall();;
  }

  public enum BallState{
    NONE,
    TOP,
    BOTTOM,
    BOTH
  }

  public static class IndexState{
    public BallState ballState;
    boolean prevDetected = false;
    boolean currDetected = true;

    public IndexState(BallState startingState){
      this.ballState = startingState;
    }

    public void update(boolean newDetected){
      prevDetected = currDetected;
      currDetected = newDetected;

      if(!prevDetected && currDetected){
        if(ballState == BallState.NONE){
          ballState = BallState.BOTTOM;
        }
        if(ballState == BallState.TOP){
          ballState = BallState.BOTH;
        }
      }
      if(prevDetected && !currDetected && ballState == BallState.BOTTOM){
        ballState = BallState.TOP;
      }
    }
    public void setState(BallState state){
      this.ballState = state;
    }
    public BallState getState(){
      return ballState;
    }
    public BallState getDesiredState(){
      if(ballState == BallState.BOTTOM){
        return BallState.TOP;
      }
      return ballState;
    }
    public int getNumber(){
      switch(ballState){
        case BOTTOM:
        case TOP:
          return 1;
        case BOTH:
          return 2;
        default:
          case NONE:
          return 0;
      }
    }
    public void removeBall(){
      if(ballState == BallState.BOTH){
        ballState = BallState.TOP;
      }
      if(ballState == BallState.TOP){
        ballState = BallState.NONE;
      }
      if(ballState == BallState.BOTTOM){
        ballState = BallState.TOP;
      }
    }
  }
}