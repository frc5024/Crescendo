package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private Shooter shooterInstance;

  public ShooterCommand() {
    // gets the shooter instance
    shooterInstance = Shooter.getInstance();
    addRequirements(shooterInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooterInstance.warmedUp()) {
      shooterInstance.setShoot();
    } else {
      cancel(); // If the shooter wasn't warmed up, cancel
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooterInstance.isLineBroken();
  }
}