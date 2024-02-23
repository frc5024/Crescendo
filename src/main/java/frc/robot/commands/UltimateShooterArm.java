package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;

public class UltimateShooterArm extends SequentialCommandGroup {

  public UltimateShooterArm(double armPosition, double armSpeed, ShooterSetpoint shooterSetpoint) {

    addCommands(
        new ArmCommand(armPosition, armSpeed),
        new WaitCommand(1.0),
        new ShooterCommand(shooterSetpoint));

  }
}
