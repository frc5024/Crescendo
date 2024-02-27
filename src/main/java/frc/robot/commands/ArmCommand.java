package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.ArmPID;
import frc.robot.subsystems.Shooter;

public class ArmCommand extends Command {

  private ArmPID arm;
  private Shooter shooter;

  private double desiredDestination;
  private ShooterSetpoint shooterSetpoint;

  public ArmCommand(double desiredDestination, ShooterSetpoint shooterSetpoint) {
    arm = ArmPID.getInstance();
    shooter = Shooter.getInstance();

    this.desiredDestination = desiredDestination;
    this.shooterSetpoint = shooterSetpoint;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setDestination(desiredDestination);
    shooter.setDesiredSetpoint(shooterSetpoint);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {

    return true;

  }
}
