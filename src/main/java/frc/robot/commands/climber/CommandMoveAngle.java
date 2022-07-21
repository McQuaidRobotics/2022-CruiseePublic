// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.climber.ClimberArm;

public class CommandMoveAngle extends CommandBase {
  private final ClimberArm arm;
  private final double angle;
  private final double angleErrorMin;
  private final CurrentLimitType currentCurrentLimitType;
  private boolean hold;

  public enum CurrentLimitType {
    ON,
    OFF,
    SMART,
    BOTH
  }
  private final double currentLimit;

  public CommandMoveAngle(ClimberArm arm, double angle, CurrentLimitType currentLimitType, double angleErrorMin){
    this(arm, angle, currentLimitType, angleErrorMin, 0);
  }

  public CommandMoveAngle(ClimberArm arm, double angle, CurrentLimitType currentLimitType, double angleErrorMin, double currentLimit){
    this.arm = arm;
    this.angle = angle;
    this.currentCurrentLimitType = currentLimitType;
    this.angleErrorMin = angleErrorMin;
    this.currentLimit = currentLimit;
    this.hold = true;
  }

  public CommandMoveAngle(ClimberArm arm, double angle, double currentLimit){
    this.arm = arm;
    this.angle = angle;
    this.currentCurrentLimitType = CurrentLimitType.ON;
    this.angleErrorMin = 1;
    this.currentLimit = currentLimit;
    this.hold = false;
  }

  @Override
  public void initialize() {
    if(currentCurrentLimitType == CurrentLimitType.SMART || currentCurrentLimitType == CurrentLimitType.BOTH){
      arm.setAngleSmartLimit(kClimb.ANGLE_SMART_CURRENT);
    }
  }

  @Override
  public void execute() {
    arm.setAngleSetpoint(angle/kClimb.CLIMB_ROTATION_TO_DEGREE);
  }

  @Override
  public void end(boolean interrupted) {
    if(!hold){
      arm.moveAnglePOut(0);
    }

    if(currentCurrentLimitType == CurrentLimitType.SMART || currentCurrentLimitType == CurrentLimitType.BOTH){
      arm.setAngleSmartLimit(150);
    }
  }

  @Override
  public boolean isFinished() {
    double angleError = arm.calculateAngleError();
    SmartDashboard.putNumber("angle Error", angleError);
    // Check for current spike
    boolean isAtStop = ((currentCurrentLimitType == CurrentLimitType.ON || currentCurrentLimitType == CurrentLimitType.BOTH)
                             && arm.getAngleCurrent() > currentLimit);
    if(isAtStop){System.out.println("Current limit reached, at stop: " + arm.getAngleCurrent());}
    return angleError < angleErrorMin || isAtStop;
  }
}
