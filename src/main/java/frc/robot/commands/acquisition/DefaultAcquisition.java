package frc.robot.commands.acquisition;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

/** An example command that uses an example subsystem. */
public class DefaultAcquisition extends CommandBase {
  private final Acquisition acquisition;
  BooleanSupplier shouldRun;
  public DefaultAcquisition(Acquisition acquisition) {
    this.acquisition = acquisition;
    shouldRun = () -> true;
    addRequirements(acquisition);
  }
  public DefaultAcquisition(Acquisition acquisition, BooleanSupplier shouldRun) {
    this.acquisition = acquisition;
    this.shouldRun = shouldRun; 
    addRequirements(acquisition);
  }

  public void execute(){
    if (acquisition.getSetpointRPM() == 0 || !shouldRun.getAsBoolean()) {
      acquisition.stopRollersByVoltage();
    } else {
      acquisition.extendArms();
      acquisition.runClosedLoopRPM();
    }
  }
}