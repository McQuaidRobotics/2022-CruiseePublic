package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private double lastRotationSpeed = 0;
    private Rotation2d setpointAngle = new Rotation2d(); // Degrees

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        double rotationSpeed = m_rotationSupplier.getAsDouble();
        // correct for angle
        if (rotationSpeed == 0){
            if (lastRotationSpeed != 0) {
                setpointAngle = m_drivetrainSubsystem.getGyroscopeRotation();
            }
            Rotation2d angleOffset = setpointAngle.minus(m_drivetrainSubsystem.getGyroscopeRotation());
            
            SmartDashboard.putNumber("Offset Angle Drives", angleOffset.getDegrees());
            SmartDashboard.putNumber("Gyro Angle Drives", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
            if(Math.abs(angleOffset.getDegrees()) > Constants.SWERVE_ALLOWED_OFFSET){
                rotationSpeed = Constants.SWERVE_CORRECTION_SPEED * (angleOffset.getDegrees()/Math.abs(angleOffset.getDegrees())) ;
            }
        }
        SmartDashboard.putNumber("Setpoint Angle Drives", setpointAngle.getDegrees());
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                rotationSpeed, 
                m_drivetrainSubsystem.getGyroscopeRotation()));
        
        lastRotationSpeed = rotationSpeed;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
