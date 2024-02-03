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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Outside", 1);
    if (Robot.visionModule.hasTarget()) {
      SmartDashboard.putNumber("inside", 2);
      var distance = Robot.visionModule.getDistance();
      var orientation = Robot.visionModule.getRotation()[2];
      var id = Robot.visionModule.getID();
      var wantedDistance = distance - 1;
      System.err.println(wantedDistance);
      if (wantedDistance < 1) {
        wantedDistance = 0;
      }

      var translation = new Translation2d(wantedDistance, 0);
      var rotation = 0;
      var headingError = orientation - Math.PI;
      headingError = headingError / Math.PI;

      frc.robot.subsystems.Swerve.getInstance().drive(translation, rotation, isFinished(), isScheduled());

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
