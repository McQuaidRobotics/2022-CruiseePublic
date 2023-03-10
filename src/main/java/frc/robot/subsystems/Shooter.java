// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kControl;


public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax motorFront;
  private final RelativeEncoder encoderFront;
  private final PIDController shooterFrontPID = new PIDController(0.087711, 0, 0);
  private final SimpleMotorFeedforward shooterFrontFF = new SimpleMotorFeedforward(0.27065, 0.13003, 0.0041837);
  private final CANSparkMax motorBack;
  private final RelativeEncoder encoderBack;
  private final PIDController shooterBackPID = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward shooterBackFF = new SimpleMotorFeedforward(0.084821, 0.12559, 0.0038655);

  private double setpointVelocityFrontRPM = 0;
  private double setpointVelocityBackRPM = 0;

  private double lastFrontAmps = 0;
  private final DoubleLogEntry shooterFrontAmpsLog;
  private double lastBackAmps;
  private final DoubleLogEntry shooterBackAmpsLog;

  public Shooter() {
    motorFront = new CANSparkMax(kCANIDs.SHOOTER_MOTOR_FRONT, MotorType.kBrushless);
    motorFront.restoreFactoryDefaults();
    motorFront.setInverted(true);
    motorFront.setIdleMode(IdleMode.kCoast);

    encoderFront = motorFront.getEncoder();

    motorBack = new CANSparkMax(kCANIDs.SHOOTER_MOTOR_BACK, MotorType.kBrushless);
    motorBack.restoreFactoryDefaults();
    motorBack.setInverted(false);
    motorBack.setIdleMode(IdleMode.kCoast);

    encoderBack = motorBack.getEncoder();

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    tab.addNumber("Setpoint RPM Front", () -> setpointVelocityFrontRPM).withPosition(0, 0).withSize(0, 0);
    tab.addNumber("Actual RPM Front", this::getVelocityFront).withPosition(1, 0).withSize(0, 0);
    tab.addNumber("Mechanism Setpoint RPM Front", () -> setpointVelocityFrontRPM * kControl.SHOOTER_FRONT_GEAR_RATIO).withPosition(2, 0).withSize(0, 0);
    tab.addNumber("Setpoint RPM Back", () -> setpointVelocityBackRPM).withPosition(0, 1).withSize(0, 0);
    tab.addNumber("Actual RPM Back", this::getVelocityBack).withPosition(1, 1).withSize(0, 0);
    tab.addNumber("Mechanism Setpoint RPM Back", () -> setpointVelocityBackRPM * kControl.SHOOTER_BACK_GEAR_RATIO).withPosition(2, 1).withSize(0, 0);

    DataLog log = Robot.getDataLog();
    shooterFrontAmpsLog = new DoubleLogEntry(log, "Shooter/Front-Amps");
    shooterBackAmpsLog = new DoubleLogEntry(log, "Shooter/Back-Amps");
  }

  public Command commandRunShooter(ShooterRPMS rpms, boolean isInstant) {
    return Commands.startEnd(
            () -> {
              setVelocityFront(rpms.RPMFront);
              setVelocityBack(rpms.RPMBack);
            },
            () -> {},
            this
    ).until(
            () -> (Math.abs(getVelocityFront() - (rpms.RPMFront / kControl.SHOOTER_FRONT_GEAR_RATIO)) < 50 && Math.abs(getVelocityBack() - rpms.RPMBack) < 50) || isInstant
    );
  }

  public Command commandStopShooter() {
    return Commands.runOnce(
            () -> {
              setVelocityFront(0);
              setVelocityBack(0);
            }
    );
  }

  /**
   * Sets the velocity for the front shooter mechanism.
   *
   * @param setpoint value to set it to
   */
  public void setVelocityFront(double setpoint) {
    setpointVelocityFrontRPM = setpoint / kControl.SHOOTER_FRONT_GEAR_RATIO;
  }

  /**
   * Sets the velocity for the back shooter mechanism.
   *
   * @param setpoint value to set it to
   */
  public void setVelocityBack(double setpoint) {
    setpointVelocityBackRPM = setpoint / kControl.SHOOTER_BACK_GEAR_RATIO;
  }

  /**
   * Gets the velocity for the front shooter motor.
   * @return velocity in RPM
   */
  public double getVelocityFront() {
    return encoderFront.getVelocity();
  }

  /**
   * Gets the velocity for the back shooter motor.
   * @return velocity in RPM
   */
  public double getVelocityBack() {
    return encoderBack.getVelocity();
  }

  @Override
  public void periodic() {
    if(motorFront.getOutputCurrent() != lastFrontAmps) shooterFrontAmpsLog.append(motorFront.getOutputCurrent());
    lastFrontAmps = motorFront.getOutputCurrent();
    if(motorBack.getOutputCurrent() != lastBackAmps) shooterBackAmpsLog.append(motorBack.getOutputCurrent());
    lastBackAmps = motorBack.getOutputCurrent();

    double outputVoltsFront = shooterFrontPID.calculate(encoderFront.getVelocity() / 60, setpointVelocityFrontRPM / 60) + shooterFrontFF.calculate(setpointVelocityFrontRPM / 60);
    if(setpointVelocityFrontRPM != 0) {
      motorFront.setVoltage(outputVoltsFront);
    } else motorFront.set(0);

    double outputVoltsBack = shooterBackFF.calculate(setpointVelocityBackRPM / 60) + shooterBackPID.calculate(encoderBack.getVelocity() / 60, setpointVelocityBackRPM / 60);
    if(setpointVelocityBackRPM != 0) {
      motorBack.setVoltage(outputVoltsBack);
    } else motorBack.set(0);
  }

  public static class ShooterRPMS{
    public double RPMFront;
    public double RPMBack;
    public ShooterRPMS(double RPMFront, double RPMBack){
      this.RPMBack = RPMBack;
      this.RPMFront = RPMFront;
    }
  }
}