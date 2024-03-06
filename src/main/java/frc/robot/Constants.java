package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
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
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

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

        // Speed we want to shoot for amp
        public static final double unjam = 0.3;

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
            speakerSetpoint(5200, 5200),
            podiumSetpoint(5200, 5200),
            ampSetpoint(1000, 1000);

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
        public static final double intakeSpeed = 0.8;
        public static final double outtakeSpeed = -0.6;
    }

    public final class KickerConstants {
        public static final int kickerMotor = 60;
        public static final double kickerSpeed = 0.5;
        public static final double kickerIntakingSpeed = 0.5;
        public static final double kickerPullbackSpeed = -0.1;
        public static final double pullbackTimer = 0.2;
    }

    public static final class ArmConstants {

        public static final int armtalonID = 7;
        public static final int armHallEffectID = 4;

        public static final double gearRatio = 1.0 / 120.0;
        public static final double kEncoderDistancePerPulse = 2048;
        public static final double kEncoderDistancePerPulseRAD = (2 * Math.PI) * gearRatio;

        public static final double intakeAngle = Units.degreesToRadians(0);

        public static final double ampPosition = Units.degreesToRadians(144);
        public static final double podiumPosition = Units.degreesToRadians(19);
        public static final double speakerPosition = Units.degreesToRadians(7);
        public static final double climbPosition = Units.degreesToRadians(90);
        public static final double zeroPosition = 0;

        public static final double midPoint = Units.degreesToRadians(45);

        public static final double intakeLimit = 0;
        public static final double UpperLimit = Units.degreesToRadians(90);

        public static final double kP = 15;
        public static final double kD = 0;

    }
}
