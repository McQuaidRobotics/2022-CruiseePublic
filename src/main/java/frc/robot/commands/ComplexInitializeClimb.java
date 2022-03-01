// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.InstantCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexInitializeClimb extends SequentialCommandGroup {
  /** Creates a new ComplexInitializeArm. */
  public ComplexInitializeClimb(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> climber.disableSoftLimits()),
      new CommandMoveReach(climber.innerArm, 4, true),
      new WaitCommand(1),
      new CommandMoveReach(climber.innerArm, -35, true),
      new CommandMoveAngle(climber.innerArm, -100, true),
      new InstantCommand(() ->  climber.zeroEncoders()),
      new CommandMoveAngle(climber.innerArm, 26, true),
      new InstantCommand(() ->  climber.zeroEncoders()),
      new InstantCommand(() -> climber.enableSoftLimits())
    );

  }
}
