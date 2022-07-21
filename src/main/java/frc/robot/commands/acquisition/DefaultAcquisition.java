package frc.robot.commands.acquisition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

/** An example command that uses an example subsystem. */
public class DefaultAcquisition extends CommandBase {
  private final Acquisition acquisition;

  public DefaultAcquisition(Acquisition acquisition) {
    this.acquisition = acquisition;
    addRequirements(acquisition);
  }

  public void execute(){
    if (acquisition.getSetpointRPM() == 0) {
      acquisition.stopRollersByVoltage();
    } else {
      acquisition.extendArms();
      acquisition.runClosedLoopRPM();
    }
  }
}