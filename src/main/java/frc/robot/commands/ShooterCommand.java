package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends InstantCommand {
  private Shooter shooterInstance;

  public ShooterCommand() {
    shooterInstance = Shooter.getInstance();
    addRequirements(shooterInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterInstance.setWarmUp();
  }
}
