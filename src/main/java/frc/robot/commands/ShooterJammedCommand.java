package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ShooterJammedCommand extends Command{
    private Shooter shooterInstance;
    private Kicker kickerInstance;

    public ShooterJammedCommand() {
        shooterInstance = Shooter.getInstance();
        kickerInstance = Kicker.getInstance();
        addRequirements(shooterInstance);
      }
    
    public void initialize() {
        shooterInstance.setJammed();
        kickerInstance.startJammed();
      }
}
