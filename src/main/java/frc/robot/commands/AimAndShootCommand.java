package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;
import frc.robot.subsystems.Shooter;

public class AimAndShootCommand extends SequentialCommandGroup {
  private final Shooter s_Shooter = Shooter.getInstance();

  public AimAndShootCommand(double armPosition, ShooterSetpoint shooterSetpoint) {
    addCommands(
        new ArmCommand(armPosition, shooterSetpoint),
        new InstantCommand(() -> s_Shooter.setWarmUp()),
        Commands.waitUntil(() -> Shooter.getInstance().warmedUp()),
        // new WaitCommand(1.0),
        new ShooterCommand());
  }
}
