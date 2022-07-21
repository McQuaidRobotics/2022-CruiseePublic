// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CommandWaitForButton extends CommandBase {
  private final XboxController gamepad;
  private final int button;

  private boolean wasReleased = false;

  public CommandWaitForButton(XboxController gamepad, int button) {
    this.gamepad = gamepad;
    this.button = button;
  }

  @Override
  public void execute() {
    if(!gamepad.getRawButton(button)){
      wasReleased = true;
    }
  }

  @Override
  public boolean isFinished() {
    return gamepad.getRawButton(button) && wasReleased;
  }
}
