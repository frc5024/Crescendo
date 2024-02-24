
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class IntakeOn extends InstantCommand {
  private Intake intake;
  private Kicker kicker;
  private Shooter shooter;

  public IntakeOn() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Intake.getInstance();
    kicker = Kicker.getInstance();
    shooter = Shooter.getInstance();
    addRequirements(intake, kicker, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startIntaking();
    kicker.startIntaking();
    shooter.setReverse();
  }
}
