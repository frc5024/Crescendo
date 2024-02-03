// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveTowardsAprilTagCommand extends Command {
  /** Creates a new DriveTowardsAprilTagCommand. */
  public DriveTowardsAprilTagCommand() {
    // Shuffleboard.getTab("example").addDouble("example", () ->
    // Robot.visionModule.getDistance() );
    // var tab = Shuffleboard.getTab("aprilTag");
    // tab.addDouble("X", () -> Robot.visionModule.getTranslation()[0]);
    // tab.addDouble("Y", () -> Robot.visionModule.getTranslation()[1]);
    // tab.addDouble("Z", () -> Robot.visionModule.getTranslation()[2]);
    // tab.addDouble("Roll", () -> Robot.visionModule.getRotation()[0]);
    // tab.addDouble("Pitch", () -> Robot.visionModule.getRotation()[1]);
    // tab.addDouble("Yaw", () -> Robot.visionModule.getRotation()[2]);
    // tab.addBoolean("shouldShoot", () -> Robot.visionModule.shouldShoot());
    // Use addRequirements() here to declare subsystem dependencies.
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.visionModule.hasTarget()) {
      var distance = Robot.visionModule.getDistance();
      var orientation = Robot.visionModule.getRotation()[2];
      var id = Robot.visionModule.getID();
      var wantedDistance = distance - 1;
      if (wantedDistance < 0) {
        wantedDistance = 0;
      }
      SmartDashboard.putNumber("Set distance", wantedDistance);

      var translation = new Translation2d(wantedDistance, 0);
      // The orientation of the AprilTag is pointe at the camera,
      // To get the error we flip it, so a 0 error will mean the camera
      // is pointed directly at the april tag
      var headingError = modpi(orientation + Math.PI);
      SmartDashboard.putNumber("Heading error", headingError);

      // The heading error will be [-pi, pi], so this is a simple P controller
      // that converts from radian error to radians per second orientation command
      // I haven't tested if the sign of this is correct WRT the swerve drive.
      var normalizedHeadingError = headingError / Math.PI;
      SmartDashboard.putNumber("NormalizedHeadingError", normalizedHeadingError);

      frc.robot.subsystems.Swerve.getInstance().drive(translation, normalizedHeadingError, isFinished(), isScheduled());

    }
    SmartDashboard.putNumber("After", 3);
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
