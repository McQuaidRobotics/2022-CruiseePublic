// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.CommandMoveAngle.CurrentLimitType;
import frc.robot.constants.kClimb;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drives.Drives;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandOnCancelClimb extends ParallelCommandGroup {
  public CommandOnCancelClimb(Climber climber, Drives drives) {
    addCommands(
      new InstantCommand(() -> drives.setRunDrives(true)),
      new CommandMoveReach(climber.innerArm, 0, true),
      new CommandMoveReach(climber.outerArm, 0, true),
      new CommandMoveAngle(climber.innerArm, 0, CurrentLimitType.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT),
      new CommandMoveAngle(climber.outerArm, 0, CurrentLimitType.OFF, kClimb.CLIMB_ANGLE_ALLOWED_ERROR_EXACT)
    );
  }
}
