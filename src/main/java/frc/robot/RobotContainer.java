package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterJammedCommand;
import frc.robot.commands.SlowCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ampShoot;
import frc.robot.commands.podiumShoot;
import frc.robot.commands.subWooferShoot;
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
    private final JoystickButton toggleOuttake = new JoystickButton(driver, XboxController.Button.kA.value);

    // opperator buttons

    private final JoystickButton subWoofer = new JoystickButton(operator, XboxController.Button.kX.value);
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
    // auto
    private final SendableChooser<Command> autoChooser;

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     * 
     */
    public RobotContainer() {
        // ...

        // Build an auto chooser. This will use Commands.none() as the default option.
        // autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        // SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> false // true = robotcentric

                ));

        // Configure the button bindings
        configureButtonBindings();
        // Command names in Path Planner
        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("Shoot",
                new ShooterCommand(Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));
        NamedCommands.registerCommand("IntakeOn", new IntakeOn());
        NamedCommands.registerCommand("IntakeOff", new IntakeOff());
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(

                s_Swerve::getPose, // Robot pose supplier
                s_Swerve::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                s_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                s_Swerve// Reference to this subsystem to set requirements

        );

        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        slowMode.whileTrue(new SlowCommand());
        toggleIntake.whileTrue(new IntakeCommand());
        toggleOuttake.whileTrue(new OuttakeCommand());

        shooterJammed.whileTrue(new ShooterJammedCommand());

        subWoofer.onTrue(new subWooferShoot());
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
        return autoChooser.getSelected();
    }

    public void autonomousInit() {
        s_Swerve.zeroHeading();
    }
}
