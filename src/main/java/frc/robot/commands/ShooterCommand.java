package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private Shooter shooterInstance;
  private ShooterSetpoint setpoint;

  public ShooterCommand(ShooterSetpoint setpoint) {
    // gets the shooter instance
    this.setpoint = setpoint;
    shooterInstance = Shooter.getInstance();
    addRequirements(shooterInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterInstance.setWarmUp();
    shooterInstance.setSetpoint(setpoint);
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