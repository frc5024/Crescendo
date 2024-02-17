package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class OuttakeCommand extends Command {
  private Intake intakeInstance;
  private Kicker kickerInstance;
  private Shooter shooterInstance;

  /** Creates a new IntakeCommand. */
  public OuttakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    // initializes subsytem instances
    intakeInstance = Intake.getInstance();
    kickerInstance = Kicker.getInstance();
    shooterInstance = Shooter.getInstance();

    addRequirements(intakeInstance, kickerInstance, shooterInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starts motors in reverse
    intakeInstance.startOuttake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stops motors
    intakeInstance.startIdle();
    kickerInstance.startIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // will not finish unless button is released
    return false;
  }
}
