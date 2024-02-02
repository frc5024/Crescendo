// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends InstantCommand {
  private Intake intakeInstance;

  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeInstance = Intake.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // makes intake function as a toggle between on or off
    if (intakeInstance.getCurrentState() == Intake.State.Idle) {
      intakeInstance.startIntaking();
    } else if (intakeInstance.getCurrentState() == Intake.State.Intaking) {
      intakeInstance.startIdle();
    }
  }

}
