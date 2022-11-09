// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.acquisition.DefaultAcquisition;
import frc.robot.commands.climber.CommandAutoClimb;
import frc.robot.commands.climber.CommandOnCancelClimb;
import frc.robot.commands.climber.DefaultClimber;
import frc.robot.commands.drives.DefaultDriveCommand;
import frc.robot.commands.index.DefaultIndex;
import frc.robot.commands.shooter.CommandRunShooter;
import frc.robot.commands.shooter.ComplexShootBalls;
import frc.robot.commands.shooter.ComplexSpinUpShooter;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kControl;
import frc.robot.constants.kSwerve;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drives.Drives;
import frc.robot.utils.AutoUtil;
import frc.robot.utils.ControllerRumble;

public class RobotContainer {
    public final PowerDistribution pdp = new PowerDistribution(kCANIDs.PDP, PowerDistribution.ModuleType.kRev);
    public final PneumaticHub pneumaticHub = new PneumaticHub(kCANIDs.PNEUMATIC_HUB);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private ControllerRumble driverControllerLeftRumble = new ControllerRumble(0, 0);
    private ControllerRumble driverControllerRightRumble = new ControllerRumble(0, 0);
    private ControllerRumble operatorControllerLeftRumble = new ControllerRumble(0, 0);
    private ControllerRumble operatorControllerRightRumble = new ControllerRumble(0, 0);

    private final Drives drives = new Drives();
    private final Acquisition acquisition = new Acquisition();
    private final Shooter shooter = new Shooter();
    private final Index index = new Index();
    private final Climber climber = new Climber();
    private final LED led = new LED();

    public RobotContainer() {
        drives.setDefaultCommand(new DefaultDriveCommand(
                drives,
                        () -> -modifyAxis(driverController.getLeftY()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getLeftX()) * kSwerve.MAX_VELOCITY_METERS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> -modifyAxis(driverController.getRightX()) * kSwerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * (1 - (modifyAxis(driverController.getLeftTriggerAxis()) * 0.9)),
                        () -> driverController.getHID().getRightStickButton()
        ));

        acquisition.setDefaultCommand(new DefaultAcquisition(acquisition, this::shouldAcquisitionRun));
        index.setDefaultCommand(new DefaultIndex(index));

        climber.setDefaultCommand(new DefaultClimber(climber,
                () -> 0 * -operatorController.getRightY() * 0.4,
                () -> 0 * operatorController.getRightX() * 0.4,
                () -> 0 * -operatorController.getLeftY() * 0.4,
                () -> 0 * operatorController.getLeftX() * 0.4 // CURRENTLY DISABLED
        ));

        configureDriverControllerBindings();
        configureOperatorControllerBindings();
    }

    private void configureDriverControllerBindings() {
        driverController.a().onTrue(new InstantCommand(drives::zeroGyroscope));
        // new Button(driverController::getStartButton);

        // Colored buttons
        driverController.a().onTrue(new ComplexShootBalls(shooter, index, acquisition, 2, kControl.SHOOTER_LOW_RPMS));
        driverController.b().onTrue(new ComplexShootBalls(shooter, index, acquisition, 2, kControl.SHOOTER_HIGH_RPMS));
        driverController.x().onTrue(new ComplexShootBalls(shooter, index, acquisition, 1, kControl.SHOOTER_LOW_RPMS));
        driverController.y().onTrue(new ComplexSpinUpShooter(shooter, acquisition, kControl.SHOOTER_HIGH_RPMS));


        // POV
        // new POVButton(driverController, 0);
        // new POVButton(driverController, 90);
        // new POVButton(driverController, 180);
        // new POVButton(driverController, 270);

        // Bumpers
        driverController.rightBumper().onTrue(new InstantCommand(() -> acquisition.setRollerRPM(kControl.ACQUISITION_RPMS)));
        driverController.leftBumper().onTrue(new InstantCommand(() -> {acquisition.setRollerRPM(0);
                                     acquisition.retractArms();
                                }));

        // Joystick Buttons
        // new Button(driverController::getRightStickButton);
        // new Button(driverController::getLeftStickButton);

        // Triggers
        // new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);
        // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
    }

    private void configureOperatorControllerBindings() {
        // Start/Back
        operatorController.start().onTrue(new InstantCommand(climber::releaseLock));
        operatorController.back().onTrue(new InstantCommand(climber::extendBreak));

        // Colored buttons
        operatorController.a().onTrue((new CommandAutoClimb(climber, drives, index, operatorController.getHID()))
                        .until(() -> operatorController.getHID().getPOV() == 0)
                );
        operatorController.b().onTrue(new ComplexShootBalls(shooter, index, acquisition, 2, kControl.SHOOTER_HIGH_RPMS));
        operatorController.x().onTrue(new ComplexShootBalls(shooter, index, acquisition, 2, kControl.SHOOTER_LOW_RPMS));
        operatorController.y().onTrue(new CommandRunShooter(shooter, kControl.SHOOTER_HIGH_RPMS, true));

        // POV
        operatorController.pov(0).onTrue(new CommandOnCancelClimb(climber, drives)); 
        operatorController.pov(90).onTrue(new InstantCommand(() -> climber.moveSidewaysPOut(-1)))
        .onFalse(new InstantCommand(() -> climber.moveSidewaysPOut(0)));
        // new POVButton(operatorController, 180);
        operatorController.pov(270).whileTrue(new InstantCommand(() -> climber.moveSidewaysPOut(1)))
                .onFalse(new InstantCommand(() -> climber.moveSidewaysPOut(0)));

        // Bumpers
        operatorController.rightBumper().onTrue(new InstantCommand(() -> acquisition.setRollerRPM(kControl.ACQUISITION_RPMS)));
        operatorController.leftBumper().whileFalse(new InstantCommand(() -> {acquisition.setRollerRPM(0);
                                     acquisition.retractArms();
                                }));

        // Joystick Buttons
        // new Button(operatorController::getRightStickButton);
        // new Button(operatorController::getLeftStickButton);

        // Triggers
        // new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
        //         .whenActive(() -> shooter.setVelocity(5200))
        //         .whenInactive(() -> shooter.setVelocity(0));
        // new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5)
                // .whenActive(() -> {});
    }

    public void applyControllerRumble() {
        driverController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, driverControllerLeftRumble.getCurrentRumble());
        driverController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, driverControllerRightRumble.getCurrentRumble());
        operatorController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, operatorControllerLeftRumble.getCurrentRumble());
        operatorController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, operatorControllerRightRumble.getCurrentRumble());
    }

    public void resetSubsystems() {
        pdp.clearStickyFaults();
        pneumaticHub.clearStickyFaults();
        drives.zeroGyroscope();
        acquisition.setRollerRPM(0);
        shooter.setVelocityFront(0);
        shooter.setVelocityBack(0);
    }

    public static double modifyAxis(double rawValue) {
        // Deadband
        double deadband = 0.05;
        double computedValue = rawValue;
        if (Math.abs(computedValue) > deadband) {
            if (computedValue > 0.0) {
                computedValue = (computedValue - deadband) / (1.0 - deadband);
            } else {
                computedValue = (computedValue + deadband) / (1.0 - deadband);
            }
        } else {
            computedValue = 0.0;
        }

        // Square the axis
        computedValue = Math.copySign(computedValue * computedValue, computedValue);

        return computedValue;
    }

    public void runAutonomousRoutine(AutoUtil.Routine routine) {
        switch (routine) {
            case HANGAR_TWO_BALL:
                new SequentialCommandGroup(
                        new InstantCommand(() -> acquisition.setRollerRPM(kControl.ACQUISITION_RPMS)),
                        AutoUtil.generateCommand("Hangar-Two-Ball-1", drives),
                        new WaitCommand(1),
                        new ComplexShootBalls(shooter, index, acquisition, 3, kControl.SHOOTER_AUTO_RPMS),
                        AutoUtil.generateCommand("Hangar-Two-Ball-2", drives)
                ).schedule();
                break;
            case TERMINAL_TWO_BALL:
                new SequentialCommandGroup(
                        new InstantCommand(() -> acquisition.setRollerRPM(kControl.ACQUISITION_RPMS)),
                        AutoUtil.generateCommand("Terminal-Two-Ball-1", drives),
                        new WaitCommand(1),
                        new ComplexShootBalls(shooter, index, acquisition, 3, kControl.SHOOTER_AUTO_RPMS),
                        AutoUtil.generateCommand("Terminal-Two-Ball-2", drives)
                ).schedule();
                break;
            case POTATO:
                new SequentialCommandGroup(
                        new ComplexShootBalls(shooter, index, acquisition, 2, kControl.SHOOTER_AUTO_RPMS),
                        AutoUtil.generateCommand("Potato", drives)
                ).schedule();
                break;
            case NOTHING:
                return;
            case DEFAULT:
                new SequentialCommandGroup(
                        AutoUtil.generateCommand("Default", drives)
                ).schedule();
                break;
            case TEST:
                new SequentialCommandGroup(
                        AutoUtil.generateCommand("Forward_4_Meters_90_Turn", drives)
                ).schedule();
                break;
        }
    }

    public void setLEDs(int pattern){
        led.arduinoPattern(pattern);
    }

    public void setDriverControllerRumble(GenericHID.RumbleType side, double amplitude, double seconds) {
        if(side == GenericHID.RumbleType.kLeftRumble) driverControllerLeftRumble = new ControllerRumble(amplitude, seconds);
        else driverControllerRightRumble = new ControllerRumble(amplitude, seconds);
    }

    public void setOperatorControllerRumble(GenericHID.RumbleType side, double amplitude, double seconds) {
        if(side == GenericHID.RumbleType.kLeftRumble) operatorControllerLeftRumble = new ControllerRumble(amplitude, seconds);
        else operatorControllerRightRumble = new ControllerRumble(amplitude, seconds);
    }

    public boolean shouldAcquisitionRun(){
        return index.getBallsIndexed() != 2; // Make sure we are not running acquisition on a stationary ball
    }
}
