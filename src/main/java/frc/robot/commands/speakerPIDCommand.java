
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class speakerPIDCommand extends PIDCommand {
  /** Creates a new PIDampCommand. */
  public speakerPIDCommand() {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {

        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
