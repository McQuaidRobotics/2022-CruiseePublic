// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber.CommandMoveAngle.CurrentLimit;
import frc.robot.constants.kClimb;
import frc.robot.utils.ClimberArm;

public class CommandMoveAngleDebounced extends CommandBase {
  /** Creates a new CommandSetReach. */
  private ClimberArm arm;
  private double angle;
  private double angleErrorMin;
  private CurrentLimit useCurrentLimits;
  private boolean hold;
  private double currentLimit;
  public CommandMoveAngleDebounced(ClimberArm arm, double angle, CurrentLimit useCurrentLimits, double angleErrorMin, double currentLimit){
    this.arm = arm;
    this.angle = angle;
    this.useCurrentLimits = useCurrentLimits;
    this.angleErrorMin = angleErrorMin;
    this.currentLimit = currentLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(useCurrentLimits == CurrentLimit.SMART || useCurrentLimits == CurrentLimit.BOTH){
      arm.setAngleSmartLimit(kClimb.ANGLE_SMART_CURRENT);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setAngleSetpoint(angle/kClimb.CLIMB_ROTATION_TO_DEGREE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!hold){
      arm.moveAnglePOut(0);
    }

    if(useCurrentLimits == CurrentLimit.SMART || useCurrentLimits == CurrentLimit.BOTH){
      arm.setAngleSmartLimit(150);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleError = arm.calculateAngleError();
    SmartDashboard.putNumber("angle Error", angleError);
    // Check for current spike
    boolean isAtStop = ((useCurrentLimits == CurrentLimit.ON || useCurrentLimits == CurrentLimit.BOTH)
                             && arm.getAngleCurrent() > currentLimit);
    if(isAtStop){System.out.println("Current limit reached, at stop: " + arm.getAngleCurrent());}
    System.out.println("Current: " + arm.getAngleCurrent());
    return angleError < angleErrorMin || arm.debounceCurrentAngle(isAtStop);
  }
}