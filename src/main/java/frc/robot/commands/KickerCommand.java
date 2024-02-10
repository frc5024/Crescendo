package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Kicker;

public class KickerCommand extends InstantCommand {
  private Kicker kickerInstance;

  public KickerCommand() {
    kickerInstance = Kicker.getInstance();
    addRequirements(kickerInstance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Switches kicker between intaking and idle, on and off
    kickerInstance.startIntaking();
  }
}
