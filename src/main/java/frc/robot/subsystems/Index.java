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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kDIO;


public class Index extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;
  private final DigitalInput beambreak;
  private int ballsIndexed = 0;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Index");
  private final DoubleLogEntry indexAmperageLog;
  private final DoubleLogEntry rotationNumberLog;

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

    tab.addNumber("Balls", this::getBallsIndexed).withPosition(0, 0).withSize(1, 1);
    tab.addBoolean("Beambreak", beambreak::get).withPosition(1, 0).withSize(1, 1);

    DataLog log = Robot.getDataLog();
    indexAmperageLog = new DoubleLogEntry(log, "Idx/Index-Amps");
    rotationNumberLog = new DoubleLogEntry(log, "Idx/Rotations");
  }

  @Override
  public void periodic() {
    indexAmperageLog.append(motor.getOutputCurrent());
    rotationNumberLog.append(getIndexPosition());
  }

  public boolean isBallBlockingBeam(){
    return !beambreak.get();
  }

  public double getIndexPosition(){
    return encoder.getPosition();
  }

  public int getBallsIndexed(){
    return ballsIndexed;
  }

  public void setBallsIndexed(int balls){
    ballsIndexed = balls;
  }

  public void runClosedLoopPosition(double rotations){
    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  }
  public void runPercentOut(double percent){
    motor.set(percent);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}