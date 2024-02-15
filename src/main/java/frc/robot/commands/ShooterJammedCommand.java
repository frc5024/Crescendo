package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterJammedCommand extends Command{
    private Shooter shooterInstance;

    public ShooterJammedCommand() {
        shooterInstance = Shooter.getInstance();
        addRequirements(shooterInstance);
      }
    
    public void initialize() {
        shooterInstance.setJammed();
      }
}
