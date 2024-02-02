package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class OuttakeCommand extends InstantCommand {
  private Intake intakeInstance;

  public OuttakeCommand() {
    intakeInstance = Intake.getInstance();
    addRequirements(intakeInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeInstance.getCurrentState() == Intake.State.Intaking) {
      intakeInstance.startOuttake();
    } else if (intakeInstance.getCurrentState() == Intake.State.Idle) {
      intakeInstance.startOuttake();
    } else if (intakeInstance.getCurrentState() == Intake.State.Outtake) {
      intakeInstance.startIdle();
    }
  }
}
