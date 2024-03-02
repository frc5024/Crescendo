package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public double speedModifier;
    public AHRS gyro;

    private static Swerve mInstance;

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    private Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();

        speedModifier = 1.0;
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = null;

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX() * speedModifier,
                    translation.getY() * speedModifier,
                    rotation,
                    getHeading());
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX() * speedModifier,
                    translation.getY() * speedModifier,
                    rotation);
        }

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        // Log desired states to AK
        Logger.recordOutput("Subsystems/SwerveDrive/DesiredSpeeds/xMPS",
                chassisSpeeds != null ? chassisSpeeds.vxMetersPerSecond : 0.0);
        Logger.recordOutput("Subsystems/SwerveDrive/DesiredSpeeds/yMPS",
                chassisSpeeds != null ? chassisSpeeds.vyMetersPerSecond : 0.0);
        Logger.recordOutput("Subsystems/SwerveDrive/DesiredSpeeds/oRPS",
                chassisSpeeds != null ? chassisSpeeds.omegaRadiansPerSecond : 0.0);
        Logger.recordOutput("Subsystems/SwerveDrive/DesiredModuleStates", swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

        // Log desired states to AK
        Logger.recordOutput("PathPlanner/Desired Module States", desiredStates);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        // negative to fix field relative
        return Rotation2d.fromDegrees(-gyro.getYaw());

    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.update();
        }

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Gyro", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());

        // Log subsystem to AK
        double[] acceleration = new double[] { this.gyro.getWorldLinearAccelX(), this.gyro.getWorldLinearAccelY() };

        Logger.recordOutput("Subsystems/SwerveDrive/Gyro/isConnected", this.gyro.isConnected());
        Logger.recordOutput("Subsystems/SwerveDrive/Gyro/PositionDeg", this.gyro.getYaw());
        Logger.recordOutput("Subsystems/SwerveDrive/Gyro/Acceleration", acceleration);
        Logger.recordOutput("Subsystems/SwerveDrive/Gyro/VelocityDegPerSec", this.gyro.getGyroFullScaleRangeDPS());
        Logger.recordOutput("Subsystems/SwerveDrive/Actual Module States", getModuleStates());
        Logger.recordOutput("Subsystems/SwerveDrive/Pose", getPose());
        Logger.recordOutput("Subsystems/SwerveDrive/ActualSpeeds/xMPS", getRobotRelativeSpeeds().vxMetersPerSecond);
        Logger.recordOutput("Subsystems/SwerveDrive/ActualSpeeds/yMPS", getRobotRelativeSpeeds().vyMetersPerSecond);
        Logger.recordOutput("Subsystems/SwerveDrive/ActualSpeeds/oRPS", getRobotRelativeSpeeds().omegaRadiansPerSecond);
    }
}