// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.climber.ClimberArm;
import frc.robot.subsystems.drives.Drives;

public class CommandLevelArm extends CommandBase {
  private final Pigeon2 pigeonTwo;
  private final ClimberArm arm;
  private final Drives drives;
  public CommandLevelArm(ClimberArm arm, Drives drives) {
    this.arm = arm;
    this.pigeonTwo = drives.getGyro();
    this.drives = drives;
  }

  @Override
  public void initialize() {
    drives.zeroGyroscope();
  }

  @Override
  public void execute() {
    double pitchError = -pigeonTwo.getRoll();
    arm.setAngleSetpoint(arm.getAngle()+pitchError);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(pigeonTwo.getRoll()) < kClimb.CLIMB_ANGLE_ALLOWED_ERROR_GENERAL;
  }
}
