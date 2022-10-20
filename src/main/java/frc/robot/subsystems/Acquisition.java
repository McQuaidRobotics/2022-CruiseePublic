package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kPneumatics;


public class Acquisition extends SubsystemBase {
  private double setpointRPM = 0;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  private final Solenoid arms = new Solenoid(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, kPneumatics.ACQ_ARMS);

  private double lastArmsAmperage;
  private final DoubleLogEntry armsAmperageLog;

  public Acquisition() {
    motor = new CANSparkMax(kCANIDs.ACQ_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);

    encoder = motor.getEncoder();
    
    pid = motor.getPIDController();
    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setFF(0.000156);
    pid.setIZone(0);
    pid.setOutputRange(-1,1);

    ShuffleboardTab tab = Shuffleboard.getTab("Acquisition");
    tab.addBoolean("Arms Extended", this::areArmsExtended).withPosition(0, 0).withSize(1, 1);
    tab.addNumber("Actual RPM", this::getRollerRPM).withPosition(1, 1).withSize(1, 1);
    tab.addNumber("Setpoint RPM", this::getSetpointRPM).withPosition(1, 0).withSize(1, 1);

    DataLog log = Robot.getDataLog();
    armsAmperageLog = new DoubleLogEntry(log, "Acq/Roller-Amps");
  }

  public void extendArms(){
    arms.set(true);
  }

  public void retractArms(){
    arms.set(false);
    setpointRPM = 0;
  }

  public boolean areArmsExtended(){
    return arms.get();
  }

  public void stopRollersByVoltage(){
    motor.setVoltage(0);
  }
  
  public void runClosedLoopRPM(){
    pid.setReference(setpointRPM, ControlType.kVelocity);
  }

  public double getSetpointRPM(){
    return setpointRPM;
  }

  public void setRollerRPM(double setpoint) {
    this.setpointRPM = setpoint;
  }

  public double getRollerRPM(){
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    if(motor.getOutputCurrent() != lastArmsAmperage) armsAmperageLog.append(motor.getOutputCurrent());
    lastArmsAmperage = motor.getOutputCurrent();
  }

  @Override
  public void simulationPeriodic() {
  }
}