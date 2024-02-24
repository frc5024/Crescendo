package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;

public class AimAndShootCommand extends SequentialCommandGroup {

  public AimAndShootCommand(double armPosition, ShooterSetpoint shooterSetpoint) {
    addCommands(
        new ArmCommand(armPosition),
        new WaitCommand(1.0),
        new ShooterCommand(shooterSetpoint));
  }
}
