package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LockOnCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SlowCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmPID;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.VisionSubsystem;

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
    // private final Joystick driver = new Joystick(0);
    // private final Joystick operator = new Joystick(1);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final Trigger zeroGyro = driver.y();
    private final Trigger slowMode = driver.x();
    private final Trigger toggleIntake = driver.rightBumper();
    private final Trigger toggleOuttake = driver.a();
    private final Trigger shoot = driver.rightTrigger();
    private final Trigger lockOn = driver.leftTrigger();

    // opperator buttons

    private final Trigger shooterWarmup = operator.rightBumper();
    private final Trigger plop = operator.povLeft();
    private final Trigger backOut = operator.povDown();

    private final Trigger ampPos = operator.b();
    private final Trigger zeroPos = operator.a();
    private final Trigger podiumPos = operator.y();
    private final Trigger speakerPos = operator.x();
    private final Trigger passingPos = operator.back();

    private final Trigger climbPos = operator.leftTrigger();

    private final Trigger climb = operator.start();

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Intake s_Intake = Intake.getInstance();
    private final Shooter s_Shooter = Shooter.getInstance();
    private final Kicker s_Kicker = Kicker.getInstance();
    private final ArmPID s_Arm = ArmPID.getInstance();
    private final LEDs s_LEDs = LEDs.getInstance();

    // private final PoseEstimatorSubsystem poseEstimatorSubsystem;
    // private final VisionSubsystem visionSubsystem;

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

        // this.visionSubsystem = new VisionSubsystem(VisionConstants.CAMERAS);
        // this.poseEstimatorSubsystem = new
        // PoseEstimatorSubsystem(s_Swerve::getModulePositions,
        // s_Swerve::getHeading, this.visionSubsystem);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false // true = robotcentric

                ));

        // Configure the button bindings
        configureButtonBindings();

        // Command names in Path Planner
        NamedCommands.registerCommand("Intake", new IntakeCommand(false));
        // NamedCommands.registerCommand("Shoot", new ShooterCommand());
        NamedCommands.registerCommand("ShootSpeaker",
                new AimAndShootCommand(Constants.ArmConstants.speakerPosition,
                        Constants.ShooterConstants.ShooterSetpoint.podiumSetpoint));
        NamedCommands.registerCommand("ShootZero",
                new AimAndShootCommand(Constants.ArmConstants.zeroPosition,
                        Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));
        NamedCommands.registerCommand("ShootPodium",
                new AimAndShootCommand(Constants.ArmConstants.podiumPosition,
                        Constants.ShooterConstants.ShooterSetpoint.podiumSetpoint));
        NamedCommands.registerCommand("AimSpeaker", new ArmCommand(Constants.ArmConstants.speakerPosition,
                Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));
        NamedCommands.registerCommand("AimPodium", new ArmCommand(Constants.ArmConstants.podiumPosition,
                Constants.ShooterConstants.ShooterSetpoint.podiumSetpoint));
        // NamedCommands.registerCommand("Shoot-Podium",
        // new AimAndShootCommand(Constants.ArmConstants.podiumPosition,
        // Constants.ShooterConstants.ShooterSetpoint.podiumSetpoint));h
        NamedCommands.registerCommand("WarmUp", new InstantCommand(() -> s_Shooter.setWarmUp()));
        NamedCommands.registerCommand("Zero", new ArmCommand(Constants.ArmConstants.zeroPosition,
                Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                s_Swerve::getPose, // Robot pose supplier
                this::setStartingPose, // Method to reset odometry (will be called if your auto has a
                // s_Swerve::setPose, // Method to reset odometry (will be called if your auto
                // has a
                // starting pose)
                s_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                s_Swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
                                              // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                                                 // in your class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to
                             // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API
                                               // for the options here
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

        // PPHolonomicDriveController.setRotationTargetOverride((Rotation2d inbound) ->
        // inbound);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto/Chooser", autoChooser);
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
        toggleIntake.whileTrue(new IntakeCommand(true));
        toggleOuttake.whileTrue(new OuttakeCommand());
        // lockTeleop.whileTrue(new LockedTelopSwerveCommand(
        // s_Swerve,
        // () -> this.poseEstimatorSubsystem.getCurrentPose(),
        // () -> this.visionSubsystem.getBestTarget(VisionConstants.DATA_FROM_CAMERA),
        // () -> this.poseEstimatorSubsystem.getCurrentPose().getRotation(),
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis)
        // ));

        lockOn.whileTrue(new LockOnCommand (() -> -driver.getRawAxis(translationAxis),() -> -driver.getRawAxis(strafeAxis)));

        /* Operator Buttons */
        // plop.whileTrue(new ShooterJammedCommand());

        backOut.whileTrue(new InstantCommand(() -> s_Shooter.setReverse()));
        shooterWarmup.onTrue(new InstantCommand(() -> s_Shooter.setWarmUp()));
        shoot.onTrue(new ShooterCommand());

        plop.whileTrue(new ArmCommand(Constants.ArmConstants.zeroPosition,
                Constants.ShooterConstants.ShooterSetpoint.plopSetpoint));

        ampPos.onTrue(new ArmCommand(Constants.ArmConstants.ampPosition,
                Constants.ShooterConstants.ShooterSetpoint.ampSetpoint));

        podiumPos.onTrue(new ArmCommand(Constants.ArmConstants.podiumPosition,
                Constants.ShooterConstants.ShooterSetpoint.podiumSetpoint));

        speakerPos.onTrue(new ArmCommand(Constants.ArmConstants.speakerPosition,
                Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));

        climbPos.onTrue(new ArmCommand(Constants.ArmConstants.climbPosition,
                Constants.ShooterConstants.ShooterSetpoint.zero));

        zeroPos.onTrue(new ArmCommand(Constants.ArmConstants.zeroPosition,
                Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));

        passingPos.onTrue(new ArmCommand(Constants.ArmConstants.passingPosition,
                Constants.ShooterConstants.ShooterSetpoint.speakerSetpoint));

        climb.onTrue(new InstantCommand(() -> s_Arm.climb()));
        climb.onFalse(new InstantCommand(() -> s_Arm.stop()));
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
        // s_Swerve.zeroHeading();
    }

    private void setStartingPose(Pose2d pose) {
        // this.poseEstimatorSubsystem.setCurrentPose(pose);
        s_Swerve.setPose(pose);
    }
}
