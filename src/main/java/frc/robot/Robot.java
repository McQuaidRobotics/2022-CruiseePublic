// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.kAuto;
import frc.robot.constants.kLED;

public class Robot extends TimedRobot {
    private static RobotContainer robotContainer;

    private static DataLog dataLog;

    private final SendableChooser<kAuto.Routine> autoChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        DataLogManager.start("/U");
        dataLog = DataLogManager.getLog();
        DriverStation.startDataLog(dataLog);

        robotContainer = new RobotContainer();

        kAuto.Routine[] routines = kAuto.Routine.values();
        for (kAuto.Routine routine : routines) {
            switch(routine.name()) {
                case "DEFAULT":
                    autoChooser.setDefaultOption(routine.name(), routine);
                case "TEST":
                    if(DriverStation.isFMSAttached()) break;
                default:
                    autoChooser.addOption(routine.name(), routine);
            }
        }
        SmartDashboard.putData("Auto Routine", autoChooser);

        robotContainer.setLEDs(0);

        SmartDashboard.putNumber("Climb Time", 45);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.setLEDs(kLED.AUTONOMOUS_ENABLED);
        robotContainer.resetSubsystems();

        kAuto.Routine chosenAuto = autoChooser.getSelected();
        if(chosenAuto == null) chosenAuto = kAuto.Routine.POTATO;
        robotContainer.runAutonomousRoutine(chosenAuto);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetSubsystems();
        robotContainer.setLEDs(kLED.TELEOP_ENABLED);
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.applyControllerRumble();

        if(DriverStation.getMatchTime() == SmartDashboard.getNumber("Climb Time", 45)) {
            robotContainer.setDriverControllerRumble(GenericHID.RumbleType.kLeftRumble, 1, 5);
            robotContainer.setOperatorControllerRumble(GenericHID.RumbleType.kLeftRumble, 1, 5);
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    public static void setLED(int pattern) {
        robotContainer.setLEDs(pattern);
    }

    public static DataLog getDataLog() {
        return dataLog;
    }
}
