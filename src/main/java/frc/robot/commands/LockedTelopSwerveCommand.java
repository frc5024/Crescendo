package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * 
 */
public class LockedTelopSwerveCommand extends Command {
    private final Swerve swerveDrive;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<PhotonTrackedTarget> targetSupplier;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final ProfiledPIDController omegaController;

    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(6.0);

    private PhotonTrackedTarget selectedTarget;
    private boolean lostTarget;

    /**
     * 
     */
    public LockedTelopSwerveCommand(Swerve swerveDrive, Supplier<Pose2d> poseProvider,
            Supplier<PhotonTrackedTarget> targetSupplier, Supplier<Rotation2d> robotAngleSupplier,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.targetSupplier = targetSupplier;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        this.omegaController = new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI,
                Constants.Swerve.angleKD, new TrapezoidProfile.Constraints(10, 10));
        this.omegaController.setTolerance(Units.degreesToRadians(2));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.selectedTarget = null;
        this.lostTarget = false;

        addRequirements(this.swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.drive(new ChassisSpeeds(), false);
        this.selectedTarget = null;

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // settings for standard telop drive
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());

        // check for loss of target
        setAndCheckTarget();
        if (this.selectedTarget == null)
            return;

        // this.omegaController.reset(robotPose.getRotation().getRadians());
        Pose2d robotPose = this.poseProvider.get();

        double goalRotation = angle.getRadians() - Units.degreesToRadians(this.selectedTarget.getYaw());
        this.omegaController.setGoal(goalRotation);

        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
        if (this.omegaController.atGoal())
            omegaSpeed = 0;

        this.swerveDrive.drive(xVelocity, yVelocity, omegaSpeed, false, true);

        Logger.recordOutput("Commands/LockedTelopSwerveCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/LockedTelopSwerveCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/LockedTelopSwerveCommand/omegaSpeed", omegaSpeed);
        Logger.recordOutput("Commands/LockedTelopSwerveCommand/angle", angle.getDegrees());
        Logger.recordOutput("Commands/LockedTelopSwerveCommand/RadiansToTarget", goalRotation);
        Logger.recordOutput("Commands/LockedTelopSwerveCommand/AngleToTarget", Units.radiansToDegrees(goalRotation));
    }

    @Override
    public void initialize() {
        this.lostTarget = false;
        setAndCheckTarget();

        Pose2d robotPose = this.poseProvider.get();
        this.omegaController.reset(robotPose.getRotation().getRadians());

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        return this.lostTarget;
    }

    /**
     * 
     */
    private void setAndCheckTarget() {
        PhotonTrackedTarget target = this.targetSupplier.get();

        if (target == null && this.selectedTarget != null) {
            this.lostTarget = true;
        } else if (this.selectedTarget != null && this.selectedTarget.getFiducialId() != target.getFiducialId()) {
            this.lostTarget = true;
        } else {
            this.selectedTarget = target;
        }
    }
}
