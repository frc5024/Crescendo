package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends InstantCommand {
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
}