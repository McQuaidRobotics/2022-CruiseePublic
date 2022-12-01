// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.index.CommandMoveIndex;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterRPMS;

public class ShooterCommands {
  public static Command complexShootBalls(Shooter shooter, Index index, Acquisition acquisition, int balls, ShooterRPMS rpms) {
    return Commands.sequence(
            acquisition.commandExtendArms(),
            acquisition.commandStopAcquisition(),
            shooter.commandRunShooter(rpms, false).withTimeout(kControl.SPINUP_TIMEOUT_SECONDS),
            new CommandMoveIndex(index, balls * kControl.INDEX_ONE_BALL_ROTATIONS),
            new InstantCommand(() -> index.removeBalls(balls)),
            shooter.commandStopShooter(),
            new InstantCommand(() -> index.runPercentOut(0)),
            acquisition.commandRunAcquisition()
    );
  }

  public static Command complexSpinUpShooter(Shooter shooter, Acquisition acquisition, ShooterRPMS rpms) {
    return Commands.sequence(
            acquisition.commandExtendArms(),
            shooter.commandRunShooter(rpms, true)
    );
  }
}
