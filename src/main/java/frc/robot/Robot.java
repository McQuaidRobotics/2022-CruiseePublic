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
import frc.robot.constants.kLED;
import frc.robot.utils.AutoUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static RobotContainer robotContainer;

    private static DataLog dataLog;

    private final SendableChooser<AutoUtil.Routine> autoChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        DataLogManager.start("/U");
        dataLog = DataLogManager.getLog();
        DriverStation.startDataLog(dataLog);

        robotContainer = new RobotContainer();

        AutoUtil.Routine[] routines = AutoUtil.Routine.values();
        for (AutoUtil.Routine routine : routines) {
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

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.    This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.    This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        robotContainer.setLEDs(kLED.AUTONOMOUS_ENABLED);
        robotContainer.resetSubsystems();

        AutoUtil.Routine chosenAuto = autoChooser.getSelected();
        if(chosenAuto == null) chosenAuto = AutoUtil.Routine.POTATO;
        robotContainer.runAutonomousRoutine(chosenAuto);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        CommandScheduler.getInstance().cancelAll();
        robotContainer.resetSubsystems();
        robotContainer.setLEDs(kLED.TELEOP_ENABLED);
    }

    /** This function is called periodically during operator control. */
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
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    public static void setLED(int pattern) {
        robotContainer.setLEDs(pattern);
    }

    public static DataLog getDataLog() {
        return dataLog;
    }
}
