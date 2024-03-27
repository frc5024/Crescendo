package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShooterJammedCommand extends Command {
    private Shooter shooterInstance;
    private Kicker kickerInstance;

    public ShooterJammedCommand() {
        // gets instances for both shooter and kicker
        shooterInstance = Shooter.getInstance();
        kickerInstance = Kicker.getInstance();
        addRequirements(shooterInstance, kickerInstance);
    }

    public void initialize() {
        // both shooter and kicker get set to their respective jammed states
        shooterInstance.setJammed();
        // kickerInstance.startJammed();
    }
}
