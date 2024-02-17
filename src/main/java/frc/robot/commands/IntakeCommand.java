package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
  private Intake intakeInstance;
  private Kicker kickerInstance;
  private Shooter shooterInstance;

  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeInstance = Intake.getInstance();
    kickerInstance = Kicker.getInstance();
    shooterInstance = Shooter.getInstance();
    addRequirements(intakeInstance, kickerInstance, shooterInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starts motors
    intakeInstance.startIntaking();
    kickerInstance.startIntaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeInstance.startIdle();
    if (!interrupted) {
      kickerInstance.startPullback();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // checks if piece is in place, linebreak in shooter set up to check for this
    return shooterInstance.isLineBroken();
  }
}
