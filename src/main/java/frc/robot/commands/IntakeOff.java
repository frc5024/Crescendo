
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class IntakeOff extends InstantCommand {
  private Intake intake;
  private Kicker kicker;

  public IntakeOff() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Intake.getInstance();
    kicker = Kicker.getInstance();
    addRequirements(intake, kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.startIdle();
    kicker.startIdle();
  }
}
