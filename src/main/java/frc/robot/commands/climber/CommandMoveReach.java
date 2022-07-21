// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.climber.ClimberArm;

public class CommandMoveReach extends CommandBase {
  /** Creates a new CommandSetReach. */
  private final ClimberArm arm;
  private final double position;
  private final boolean useCurrentLimits;
  private final double currentLimit;
  boolean hold;
  public CommandMoveReach(ClimberArm arm, double position, boolean hold){
    this.arm = arm;
    this.position = position;
    this.hold = hold;
    this.useCurrentLimits = false;
    this.currentLimit = 0;
  }
  public CommandMoveReach(ClimberArm arm, double position, boolean hold, double currentLimit){
    this.arm = arm;
    this.position = position;
    this.hold = hold;
    this.useCurrentLimits = true;
    this.currentLimit = currentLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setReachSetpoint(position/kClimb.CLIMB_ROTATION_TO_INCH);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!hold){
      arm.moveReachPOut(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double reachError = arm.calculateReachError();
    SmartDashboard.putNumber("Reach Error", reachError);
    // Check for current spike
    boolean isAtStop = (useCurrentLimits && arm.getReachCurrent() > currentLimit);
    if(isAtStop){System.out.println("Current limit reached, at stop: " + arm.getReachCurrent());}
    System.out.println("Current: " + arm.getReachCurrent());
    return reachError < kClimb.CLIMB_REACH_ALLOWED_ERROR || isAtStop;
  }
}
