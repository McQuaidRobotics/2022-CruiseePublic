// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRPMS;

public class CommandRunShooter extends CommandBase {
  /** Creates a new CommandRunShooter. */
  Shooter shooter;
  double RPMFront;
  double RPMBack;
  boolean isInstant;

  public CommandRunShooter(Shooter shooter, ShooterRPMS rpms) {
    this(shooter, rpms, false);
  }

  public CommandRunShooter(Shooter shooter, ShooterRPMS rpms, boolean isInstant) {
    this.shooter = shooter;
    this.RPMFront = rpms.RPMFront;
    this.RPMBack = rpms.RPMBack;
    this.isInstant = isInstant;
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("shooter/isRPM", false);
  }

  @Override
  public void execute() {
    shooter.setVelocityFront(RPMFront);
    shooter.setVelocityBack(RPMBack);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("shooter/isRPM", true);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(shooter.getVelocityFront() - (RPMFront / kControl.SHOOTER_FRONT_GEAR_RATIO)) < 50 && Math.abs(shooter.getVelocityBack() - RPMBack) < 50) || isInstant;
  }
}
