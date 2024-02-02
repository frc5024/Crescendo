package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends InstantCommand {
  private Intake intakeInstance;

  public IntakeCommand() {
    intakeInstance = Intake.getInstance();
    addRequirements(intakeInstance);
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
