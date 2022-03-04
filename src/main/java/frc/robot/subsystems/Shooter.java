// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax.ControlType;

import javax.sql.rowset.serial.SerialArray;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;


public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax motor;
  public RelativeEncoder encoder;
  public SparkMaxPIDController pid;
  private double setpointVelocity = 0;
  public Shooter() {
    motor = new CANSparkMax(kCANIDs.SHOOTER_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);

    encoder = motor.getEncoder();

    pid = motor.getPIDController();
    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setFF(0.000156);
    pid.setIZone(0);
    pid.setOutputRange(-1,1);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/actual Velocity Shooter", encoder.getVelocity());
      setpointVelocity = SmartDashboard.getNumber("shooter/setpoint Velocity Shooter", setpointVelocity);
      pid.setReference(setpointVelocity, ControlType.kVelocity);
  }

  public void setVelocity(double inputVelocity){
    setpointVelocity = inputVelocity;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}