package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPID;

public class ArmCommand extends Command {

  private ArmPID arm;
  private double desiredDestination;

  public ArmCommand(double desiredDestination) {

    arm = ArmPID.getInstance();

    this.desiredDestination = desiredDestination;
  }

  @Override
  public void initialize() {
    arm.setDestination(desiredDestination);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {

    return true;

  }
}
