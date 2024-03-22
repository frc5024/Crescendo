package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.google.common.base.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class LockOntoApriltagCommand extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private final TeleopSwerve swerveDrive;
    private final Supplier <Pose2d> poseProvider;
    private final Supplier <PhotonTrackedTarget> targetSupplier;
    private final Supplier <Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final ProfiledPIDController omegaController = RobotConstants.omegaController;

    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(6.0);

    private PhotonTrackedTarget selectedTarget;
    private boolean lostTarget;

    
   

    public LockOntoApriltagCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, TeleopSwerve swerveDrive, Supplier <Pose2d> poseProvider, Supplier <PhotonTrackedTarget> targetSupplier, Supplier <Rotation2d> robotAngleSupplier, DoubleSupplier translationYSupplier, DoubleSupplier translationXSupplier) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.targetSupplier = targetSupplier;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;

        this.omegaController.setTolerance(Units.degreesToRadians(2));
        this.omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.selectedTarget = null;
        this.lostTarget = false;

       
    }
 @Override
    public void end(boolean interrupted) {
       // this.swerveDrive.stop();
        this.selectedTarget = null;

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        // settings for standard telop drive
        Rotation2d angle = this.robotAngleSupplier.get();
        double xVelocity = this.translateXRateLimiter.calculate(translationXSupplier.getAsDouble());
        double yVelocity = this.translateYRateLimiter.calculate(translationYSupplier.getAsDouble());
        var Velocity = new Translation2d(xVelocity, yVelocity);

        // check for loss of target
        setAndCheckTarget();
        if (this.selectedTarget == null) return;

        // this.omegaController.reset(robotPose.getRotation().getRadians());
        Pose2d robotPose = this.poseProvider.get();

        double goalRotation = angle.getRadians() - Units.degreesToRadians(this.selectedTarget.getYaw());
        this.omegaController.setGoal(goalRotation);
        
        double omegaSpeed = this.omegaController.calculate(robotPose.getRotation().getRadians());
        if (this.omegaController.atGoal()) omegaSpeed = 0;

        this.s_Swerve.drive(Velocity, omegaSpeed, false, true);

        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/omegaSpeed", omegaSpeed);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/angle", angle.getDegrees());
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/RadiansToTarget", goalRotation);
        Logger.recordOutput("Commands/LockedTelopDriveByPoseCommand/AngleToTarget", Units.radiansToDegrees(goalRotation));

          /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
    
    @Override
    public void initialize() {
        this.lostTarget = false;
        setAndCheckTarget();

        Pose2d robotPose = this.poseProvider.get();
        this.omegaController.reset(robotPose.getRotation().getRadians());

        //LEDController.set(this.selectedTarget == null ? LEDPreset.Solid.kRed : LEDPreset.Solid.kGreen);

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
        }
        else if (this.selectedTarget != null && this.selectedTarget.getFiducialId() != target.getFiducialId()) {
            this.lostTarget = true;
        }
        else {
            this.selectedTarget = target;
        }
    }
}

 