// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class DefaultClimber extends CommandBase {
    /** Creates a new DefaultClimber. */
    private Climber climber;
    private DoubleSupplier innerReach, innerAngle, outerReach, outerAngle;
    public DefaultClimber(Climber climber,
                            DoubleSupplier innerReach, DoubleSupplier innerAngle,
                            DoubleSupplier outerReach, DoubleSupplier outerAngle) {
        this.climber = climber;
        addRequirements(climber);
        this.innerReach = innerReach;
        this.innerAngle = innerAngle;
        this.outerReach = outerReach;
        this.outerAngle = outerAngle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.releaseLock();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.innerArm.moveReachPOut(innerReach.getAsDouble());
        climber.innerArm.moveAnglePOut(innerAngle.getAsDouble());
        climber.outerArm.moveReachPOut(outerReach.getAsDouble());
        climber.outerArm.moveAnglePOut(outerAngle.getAsDouble());
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
