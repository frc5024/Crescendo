package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.Shooter;

public class WaitForWarmUpAndShoot extends SequentialCommandGroup {
  private final Shooter s_Shooter = Shooter.getInstance();

  public WaitForWarmUpAndShoot(double armPosition, ShooterSetpoint shooterSetpoint) {
    addCommands(
        Commands.waitUntil(() -> Shooter.getInstance().warmedUp()),
        // new WaitCommand(1.0),
        new ShooterCommand());
  }
}
