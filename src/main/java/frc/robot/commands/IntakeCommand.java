package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    private Intake intakeInstance;
    private Kicker kickerInstance;
    private Shooter shooterInstance;
    private ArmPID armInstance;
    private boolean safety;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(boolean safety) {
        // Use addRequirements() here to declare subsystem dependencies.
        intakeInstance = Intake.getInstance();
        kickerInstance = Kicker.getInstance();
        shooterInstance = Shooter.getInstance();
        armInstance = ArmPID.getInstance();
        this.safety = safety;

        // arm is NOT a requirment as it is not being moved, just checking the angle
        addRequirements(intakeInstance, kickerInstance, shooterInstance);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // starts motors
        if (!safety || armInstance.getMeasurement() <= (Constants.ArmConstants.zeroPosition
                + Units.degreesToRadians(Constants.IntakeConstants.armPosMarginError))) {
            intakeInstance.startIntaking();
            kickerInstance.startIntaking();
            shooterInstance.setReverse();
        }
        /*
         * intakeInstance.startIntaking();
         * kickerInstance.startIntaking();
         * shooterInstance.setReverse();
         */

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeInstance.startIdle();
        // if (!interrupted) {
        // kickerInstance.startPullback();
        // } else {
        // kickerInstance.startIdle();
        // }
        kickerInstance.startIdle();
        shooterInstance.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // checks if piece is in place, linebreak in shooter set up to check for this
        return shooterInstance.isLineBroken();
    }
}
