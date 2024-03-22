// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class DriveTowardsAprilTagCommand extends Command {
  /** Creates a new DriveTowardsAprilTagCommand. */
  private double headingError;

  public DriveTowardsAprilTagCommand() {

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
  private boolean isAmpId(double id){
    if ( id == VisionConstants.BLUE_AMP_ID || id == VisionConstants.RED_AMP_ID)  {
      return true;
    }
    return false;
  }
  public double getHeadingError () {
    return this.headingError;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.visionModule.hasTarget()) {
      var id = Robot.visionModule.getID();
      if (isAmpId(id)) {
      var distance = Robot.visionModule.getDistance();
      var translation = Robot.visionModule.getTranslation();
      SmartDashboard.putNumber("Distance", distance);
      var orientation = Robot.visionModule.getRotation();
      var yaw = orientation [2];
      var yawError = modpi(yaw + Math.PI);
      this.headingError = yawError; 
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
