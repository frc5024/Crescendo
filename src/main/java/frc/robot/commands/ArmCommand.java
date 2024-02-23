package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmPID;

public class ArmCommand extends Command {

  private ArmPID arm;
  private double desiredDestination;
  private double desiredSpeed;

  public ArmCommand(double desiredDestination, double desiredSpeed) {

    arm = ArmPID.getInstance();

    this.desiredDestination = desiredDestination;
    this.desiredSpeed = desiredSpeed;

  }

  @Override
  public void initialize() {
    arm.setStuff(desiredDestination, desiredSpeed);
    System.out.println("AAAAAAAAAAAAAAAAAAAAAA");

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
    // return arm.getController().atSetpoint();

  }
}
