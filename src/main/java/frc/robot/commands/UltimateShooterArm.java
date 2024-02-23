package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class UltimateShooterArm extends SequentialCommandGroup {

  public UltimateShooterArm(double armPosition, double armSpeed) {

    addCommands(
        new ArmCommand(armPosition, armSpeed),
        new ShooterCommand());

  }
}
