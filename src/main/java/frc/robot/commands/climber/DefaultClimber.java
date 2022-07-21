// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.Climber;

import java.util.function.DoubleSupplier;

public class DefaultClimber extends CommandBase {
    /** Creates a new DefaultClimber. */
    private final Climber climber;
    private final DoubleSupplier innerReach;
    private final DoubleSupplier innerAngle;
    private final DoubleSupplier outerReach;
    private final DoubleSupplier outerAngle;
    private boolean isCoast;
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
        this.isCoast = false;
        SmartDashboard.putBoolean("isCoast", isCoast);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.innerArm.moveReachPOut(RobotContainer.modifyAxis(innerReach.getAsDouble()));
        climber.innerArm.moveAnglePOut(RobotContainer.modifyAxis(innerAngle.getAsDouble()));
        climber.outerArm.moveReachPOut(RobotContainer.modifyAxis(outerReach.getAsDouble()));
        climber.outerArm.moveAnglePOut(RobotContainer.modifyAxis(outerAngle.getAsDouble()));
        isCoast = SmartDashboard.getBoolean("isCoast", isCoast);
        if(isCoast) {
            climber.setToCoast();
        } else {
            climber.setToBrake();
        }
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
