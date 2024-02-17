package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterJammedCommand;
import frc.robot.commands.SlowCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ampShoot;
import frc.robot.commands.podiumShoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton strafeLeft = new JoystickButton(driver,
    // XboxController.Button.kLeftBumper.value);
    // private final JoystickButton strafeRight = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    private final JoystickButton toggleIntake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton toggleOuttake = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // opperator buttons

    private final JoystickButton shoot = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton shooterJammed = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

    private final JoystickButton ampPos = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton zeroPos = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton podiumPos = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton climbPos = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();
    private final Kicker s_Kicker = Kicker.getInstance();

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * 
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> true // true = robotcentric

                ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     *
     *
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        slowMode.onTrue(new SlowCommand());
        toggleIntake.whileTrue(new IntakeCommand());
        // toggleIntake.onFalse(new IntakeIdle());
        // toggleIntake.onTrue(new KickerCommand());
        toggleOuttake.whileTrue(new OuttakeCommand());
        // toggleOuttake.onFalse(new IntakeIdle());
        shoot.onTrue(new ShooterCommand());
        shooterJammed.whileTrue(new ShooterJammedCommand());

        ampPos.onTrue(new ampShoot());

        podiumPos.onTrue(new podiumShoot());

        climbPos.onTrue(new ArmCommand(Constants.ArmConstants.climbPosition));

        zeroPos.onTrue(new ArmCommand(Constants.ArmConstants.zeroPosition));

    }

    public void resetSubsystems() {
        s_Shooter.reset();
        s_Kicker.reset();

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
