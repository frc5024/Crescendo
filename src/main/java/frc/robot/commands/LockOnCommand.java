// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

public class LockOnCommand extends Command {
    /** Creates a new DriveTowardsAprilTagCommand. */
    private double headingError;
    private Swerve s_Swerve;
    private DoubleSupplier translationAxis;
    private DoubleSupplier strafeAxis;

    public LockOnCommand(DoubleSupplier translation, DoubleSupplier strafe) {
        this.s_Swerve = Swerve.getInstance();
        this.translationAxis = translation;
        this.strafeAxis = strafe;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Wrap an angle to [-pi pi]
    // @param in_angle in radians
    // @return wrapped angle in radians
    private double modpi(double in_angle) {
        var PI2 = Math.PI * 2;
        var angle = ((in_angle % PI2) + PI2) % PI2;
        while (angle > Math.PI) {
            angle -= PI2;
        }
        return angle;
    }

    private boolean isAmpId(double id) {
        return id == VisionConstants.BLUE_AMP_ID || id == VisionConstants.RED_AMP_ID;
    }

    private boolean isSpeakerId(double id) {
        return id == VisionConstants.BLUE_SPEAKER_ID || id == VisionConstants.RED_SPEAKER_ID;
    }

    public double getHeadingError() {
        return this.headingError;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Robot.visionModule.hasTarget()) {
            var ids = Robot.visionModule.getID();
            for (Integer id : ids) {
                if (isAmpId(id)) {
                    var orientation = Robot.visionModule.getRotation(id);
                    // var yaw = orientation[2];
                    var yawError = modpi(orientation + Math.PI);
                    // this.headingError = yawError;

                    // var neededRotation = modpi(Robot.visionModule.getRotation());
                    var neededRotation = yawError + VisionConstants.AMP_HEADING_OFFSET_RAD;
                    if (neededRotation > 0.1 || neededRotation < -0.1) {
                        s_Swerve.drive(translationAxis.getAsDouble(), strafeAxis.getAsDouble(),
                                neededRotation / 5, false,
                                false);
                    }
                }
                if (isSpeakerId(id)) {
                    var translation = Robot.visionModule.getTranslation(id);
                    var orientation = Robot.visionModule.getRotation(id);
                    // var yaw = orientation[2];
                    // var yawError = modpi(orientation + Math.PI);
                    var x = translation[0];
                    var y = translation[1];
                    var yawError = Math.atan2(y, x);
                    // this.headingError = yawError;

                    // var neededRotation = modpi(Robot.visionModule.getRotation());
                    var neededRotation = yawError + VisionConstants.SPEAKER_HEADING_OFFSET_RAD;
                    if (neededRotation > 0.1 || neededRotation < -0.1) {
                        s_Swerve.drive(translationAxis.getAsDouble(), strafeAxis.getAsDouble(),
                                neededRotation / 5, false,
                                false);
                    }
                }
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}