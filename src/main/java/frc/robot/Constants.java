package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.camera.Camera;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static class LEDConstants {
        public static final int ledPort = 8;
    }

    public static final double stickDeadband = 0.1;

    public static final class AdvantageKit {
        public enum Mode {
            REAL, REPLAY, SIM
        }

        private enum RobotType {
            ROBOT_2024, ROBOT_REPLAYBOT
        }

        // TODO: Change this when running on real/replay robot
        private static final RobotType ROBOT = RobotType.ROBOT_2024;

        public static boolean TUNING_MODE = false;

        public static Mode getMode() {
            if (RobotBase.isReal()) {
                return Mode.REAL;
            }

            return ROBOT == RobotType.ROBOT_2024 ? Mode.SIM : Mode.REPLAY;
        }
    }

    public static final class Swerve {
        public static final int AHRS = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.5); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(18.5); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.112; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.32); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51);
        public static final double driveKA = (0.27);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.0; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 9.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(154.384); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-140.9765625 + 180); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-68.7304); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(158.642578125 + 180); //
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 11;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class SlowConstants {

        // Double values for 100% and 30%
        public static final double oneHundredPercentModifier = 1.00;
        public static final double thirtyPercentModifier = 0.30;

    }

    public static final class ShooterConstants {
        public static final int leftMotorId = 62;
        public static final int rightMotorId = 61;

        // Speed we want to shoot for plop
        public static final double unjam = 0.25;

        // Warmup Jam Detection
        public static final double warmupJamPullbackTime = 0.5; // How long before attempting a pullback
        public static final double warmupJamVelocityThreshold = 500; // Minimum velocity to assume not jammed

        // PID Tuning Constants
        public static final double kP = 0.000012;
        public static final double kD = 0.0002;
        public static final double kF = 0.000172;

        public static final int autoShootSampleCount = 10;
        public static final double autoShootRPMTolerance = 0.95;

        public static final double maxOutput = 1.0;
        public static final int currentLimitAmps = 20;
        public static final double gearRatio = 1.0;

        public enum ShooterSetpoint {
            zero(0, 0),
            plopSetpoint(1500, 1500),
            speakerSetpoint(5200, 5200),
            podiumSetpoint(5200, 5200),
            ampSetpoint(2000, 2000);

            private final double leftVelocity;
            private final double rightVelocity;

            ShooterSetpoint(double leftVelocity, double rightVelocity) {
                this.leftVelocity = leftVelocity;
                this.rightVelocity = rightVelocity;
            }

            public double getLeftVelocity() {
                return leftVelocity;
            }

            public double getRightVelocity() {
                return rightVelocity;
            }
        }
    }

    public final class IntakeConstants {
        public static final int topRollerChannel = 10;
        public static final double intakeSpeed = 0.75;
        public static final double outtakeSpeed = -0.6;
        public static final double armPosMarginError = 2;
    }

    public final class KickerConstants {
        public static final int kickerMotor = 60;
        public static final double kickerSpeed = 0.5;
        public static final double kickerIntakingSpeed = 0.3;
        public static final double kickerPullbackSpeed = -0.1;
        public static final double pullbackTimer = 0.1;
    }

    public static final class ArmConstants {

        public static final int armtalonID = 7;
        public static final int armHallEffectID = 9;

        public static final double gearRatio = 1.0 / 120.0;
        public static final double kEncoderDistancePerPulse = 2048;
        public static final double kEncoderDistancePerPulseRAD = (2 * Math.PI) * gearRatio;

        public static final double intakeAngle = Units.degreesToRadians(0);

        public static final double ampPosition = Units.degreesToRadians(120);
        public static final double podiumPosition = Units.degreesToRadians(29);
        public static final double speakerPosition = Units.degreesToRadians(12);
        public static final double climbPosition = Units.degreesToRadians(95);
        public static final double zeroPosition = 0;

        public static final double midPoint = Units.degreesToRadians(45);

        public static final double intakeLimit = 0;
        public static final double UpperLimit = Units.degreesToRadians(90);

        public static final double kP = 15;
        public static final double kI = 0;
        public static final double kD = 0;

    }

    /**
     * 
     */
    public static final class RobotConstants {
        public static final double ROBOT_LENGTH = Units.inchesToMeters(38.0);
    }

    /**
     * 
     */
    public static class FieldConstants {
        // Page 4 & 5 of Layout & Marking Diagram manual
        // (https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf)
        public static final double LENGTH_METERS = Units.inchesToMeters(652.73);
        public static final double WIDTH_METERS = Units.inchesToMeters(323.0);

        // Start center of robot 1.0 meters from wall)
        public static final double POSE_X = Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2);
        public static final double[] POSE_Y = {
                Units.inchesToMeters(279.13),
                Units.inchesToMeters(218.42),
                Units.inchesToMeters(62.64)
        };

        // public static final Pose2d[] ALLIANCE_POSES = new Pose2d[] {
        // new Pose2d(POSE_X, POSE_Y[0], Rotation2d.fromDegrees(0)),
        // new Pose2d(POSE_X, POSE_Y[1], Rotation2d.fromDegrees(0)),
        // new Pose2d(POSE_X, POSE_Y[2], Rotation2d.fromDegrees(0))
        // };

        // public static final Pose2d[] SPEAKER_POSES = new Pose2d[] {
        // new Pose2d(Units.inchesToMeters(36.17 / 2) + (RobotConstants.ROBOT_LENGTH /
        // 4), Units.inchesToMeters(218.42 + 31.3) + (RobotConstants.ROBOT_LENGTH / 4),
        // Rotation2d.fromDegrees(60.0)),
        // new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2),
        // Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
        // new Pose2d(Units.inchesToMeters(36.17 / 2) + (RobotConstants.ROBOT_LENGTH /
        // 4), Units.inchesToMeters(218.42 - 31.3) - (RobotConstants.ROBOT_LENGTH / 4),
        // Rotation2d.fromDegrees(-60.0))
        // };

        public static final Pose2d AUTONOMOUS_SHOOTING_POSE = new Pose2d(3.90, 5.54, Rotation2d.fromDegrees(0.0));

        public static final Pose2d[] SPEAKER_POSES = new Pose2d[] {
                new Pose2d(0.73, 6.76, Rotation2d.fromDegrees(60.0)),
                new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2),
                        Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
                new Pose2d(0.73, 4.29, Rotation2d.fromDegrees(-60.0))
        };

        public static final Pose2d[] TUNING_POSES = new Pose2d[] {
                new Pose2d(1.00, 7.00, Rotation2d.fromDegrees(0.0)),
                new Pose2d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2),
                        Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)),
                new Pose2d(0.73, 4.29, Rotation2d.fromDegrees(-60.0))
        };

        // relative to rear camera
        public static final Translation3d[] SPEAKER_POSE_TRANSLATIONS = new Translation3d[] {
                new Translation3d(Units.inchesToMeters(36.17 / 2),
                        Units.inchesToMeters(-31.3) - (RobotConstants.ROBOT_LENGTH / 2), 0.0),
                new Translation3d(Units.inchesToMeters(36.17) + (RobotConstants.ROBOT_LENGTH / 2), 0.0, 0.0),
                new Translation3d(Units.inchesToMeters(36.17 / 2),
                        Units.inchesToMeters(31.3) + (RobotConstants.ROBOT_LENGTH / 2), 0.0)
        };

        // relative to rear camera
        public static final Rotation3d[] SPEAKER_POSE_ROTATIONS = new Rotation3d[] {
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(-60)),
                new Rotation3d(0.0, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(60))
        };

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField
                .loadAprilTagLayoutField();
    }

    /**
     * 
     */
    public static final class VisionConstants {
        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

        public static final Camera FRONT_CAMERA = new Camera("Arducam_OV9281-2",
                Camera.Type.APRILTAG, 0,
                Units.inchesToMeters(-10.0), Units.inchesToMeters(-6.25), Units.inchesToMeters(23.75),
                0.0, Units.degreesToRadians(24), 0.0);
        public static final Camera REAR_CAMERA = new Camera("Arducam_OV9281-1",
                Camera.Type.APRILTAG, 0,
                Units.inchesToMeters(-12.0), Units.inchesToMeters(-6.25), Units.inchesToMeters(23.75),
                0.0, Units.degreesToRadians(-35), Math.PI);
        public static final Camera NOTE_CAMERA = new Camera("WebCam",
                Camera.Type.COLOURED_SHAPE, 0,
                RobotConstants.ROBOT_LENGTH / 2, 0.0, Units.inchesToMeters(16),
                0.0, Units.degreesToRadians(10), 0.0);

        public static final List<Camera> CAMERAS = Arrays.asList(REAR_CAMERA);
        public static final String DATA_FROM_CAMERA = REAR_CAMERA.getName();

        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        // for simulation only
        public static final double DIAGONAL_FOV = 70;
        public static final int IMG_WIDTH = 800;
        public static final int IMG_HEIGHT = 600;
    }
}
