package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeIdle extends InstantCommand {
  private Intake intakeInstance;

  public IntakeIdle() {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeInstance = Intake.getInstance();
    addRequirements(intakeInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeInstance.startIdle();
  }
}
