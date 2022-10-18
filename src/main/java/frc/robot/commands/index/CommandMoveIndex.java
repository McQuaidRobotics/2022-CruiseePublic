// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Index;

public class CommandMoveIndex extends CommandBase {
  Index index;
  double setpoint;
  double rotations;
  int balls = 0;
  public CommandMoveIndex(Index index, double rotations) {
    this.index = index;
    this.rotations = rotations;

    addRequirements(index);
  }

  @Override
  public void initialize() {
    this.setpoint = index.getIndexPosition() + this.rotations;
  }

  @Override
  public void execute() {
    index.runClosedLoopPosition(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    return Math.abs(index.getIndexPosition()-setpoint) < kControl.INDEX_ALLOWED_ERROR_ROTATIONS;
  }
}
