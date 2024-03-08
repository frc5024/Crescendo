package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class TrapCommand extends Command {
    private Shooter shooterInstance;
    private Kicker kickerInstance;

    public TrapCommand() {
        //gets the shooter and kicker instances
        shooterInstance = Shooter.getInstance();
        kickerInstance = Kicker.getInstance();
    }

    @Override
    public void initialize(){
        shooterInstance.setIntake();
        kickerInstance.startPushing();
    }
}
