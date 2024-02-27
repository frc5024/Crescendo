// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlowCommand extends Command {

  private Swerve s_Swerve;

  public SlowCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = Swerve.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Set speed to 30%
    s_Swerve.speedModifier = Constants.SlowConstants.thirtyPercentModifier;

  }

  @Override
  public void end(boolean interupted) {
    // Set speed to 100%
    s_Swerve.speedModifier = Constants.SlowConstants.oneHundredPercentModifier;
  }

  @Override
  public boolean isFinished() {

    return false;

  }
}
