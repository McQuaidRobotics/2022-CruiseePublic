// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.kControl;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Index.BallState;

/** An example command that uses an example subsystem. */
public class DefaultIndex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Index index;
  private final double minIndexIncrement = 5;
  private double startingPosition;
  private double wantedPosition;
  private double offset = 0;
  public DefaultIndex(Index index) {
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = index.getIndexPosition();
    wantedPosition = startingPosition;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    index.update(index.isBallBlockingBeam());
    if(index.getState() == BallState.NONE){
      index.runPercentOut(0.1);
      startingPosition = index.getIndexPosition(); // always be moving one position, keeps balls from getting stuck
      wantedPosition = startingPosition;
      offset = 0;
      
    }
    if(index.getDesiredState() == BallState.TOP && index.wantsDifferentState()){
      // Shift the indexer
      wantedPosition = startingPosition + offset;
      index.runClosedLoopPosition(wantedPosition);
      if(Math.abs(wantedPosition-index.getIndexPosition()) < kControl.INDEX_ALLOWED_ERROR_ROTATIONS){
        offset+=minIndexIncrement; // Needs to be enough that it will moveS
      }
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
