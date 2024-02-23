package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ampShoot extends SequentialCommandGroup {
  /** Creates a new armShooter. */
  public ampShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmCommand(Constants.ArmConstants.ampPosition),
        new WaitCommand(1),
        new ShooterCommand(Constants.ShooterConstants.ShooterSetpoint.ampSetpoint));
  }
}
