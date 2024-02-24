package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private DigitalInput linebreak;

    private Kicker kickerInstance;

    private ShooterSetpoint setpoint;

    // LinearFilters to calculate the average RPM of the encoders
    private LinearFilter leftFilter = LinearFilter.movingAverage(Constants.ShooterConstants.autoShootSampleCount);
    private LinearFilter rightFilter = LinearFilter.movingAverage(Constants.ShooterConstants.autoShootSampleCount);

    public static final Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    public enum State {
        Idle,
        Warming,
        Shoot,
        Jammed,
        Reverse
    }

    protected StateMachine<State> stateMachine;

    private Shooter() {
        linebreak = new DigitalInput(8);

        leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorId, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();

        leftMotor.getEncoder().setVelocityConversionFactor(1.0);
        leftMotor.getEncoder().setPositionConversionFactor(1.0 / Constants.ShooterConstants.gearRatio);

        rightMotor.getEncoder().setVelocityConversionFactor(1.0);
        rightMotor.getEncoder().setPositionConversionFactor(1.0 / Constants.ShooterConstants.gearRatio);

        leftMotor.setSmartCurrentLimit(Constants.ShooterConstants.currentLimitAmps);
        rightMotor.setSmartCurrentLimit(Constants.ShooterConstants.currentLimitAmps);

        var pidController = leftMotor.getPIDController();
        pidController.setP(Constants.ShooterConstants.kP);
        pidController.setD(Constants.ShooterConstants.kD);
        pidController.setFF(Constants.ShooterConstants.kF);
        pidController.setOutputRange(-Constants.ShooterConstants.maxOutput, Constants.ShooterConstants.maxOutput);

        pidController = rightMotor.getPIDController();
        pidController.setP(Constants.ShooterConstants.kP);
        pidController.setD(Constants.ShooterConstants.kD);
        pidController.setFF(Constants.ShooterConstants.kF);
        pidController.setOutputRange(-Constants.ShooterConstants.maxOutput, Constants.ShooterConstants.maxOutput);

        kickerInstance = Kicker.getInstance();

        // shuffleboard tabs: velocity for both encoders and the linebreak
        var tab = Shuffleboard.getTab("Shooter");
        tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        tab.addBoolean("Linebroken", () -> linebreak.get());

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
        stateMachine.addState(State.Reverse, this::handleReverseState);

        SmartDashboard.putNumber("Shooter kP", 0.000012);
        SmartDashboard.putNumber("Shooter kD", 0.002);
        SmartDashboard.putNumber("Shooter kF", 0.000172);
        SmartDashboard.putNumber("Shooter max output", 1);
        SmartDashboard.putNumber("Shooter setpoint (RPM)", 0);

        SmartDashboard.putNumber("Shooter Average RPM (Left)", 0);
        SmartDashboard.putNumber("Shooter Average RPM (Right)", 0);
    }

    public void setTargetVelocity(ShooterSetpoint setpoint) {
        leftMotor.getPIDController().setReference(setpoint.getLeftVelocity(), ControlType.kVelocity);
        rightMotor.getPIDController().setReference(setpoint.getRightVelocity(), ControlType.kVelocity);
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            setTargetVelocity(ShooterSetpoint.zero);
        }
    }

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            setTargetVelocity(setpoint);
            leftFilter.reset();
            rightFilter.reset();
        }

        var leftAverage = leftFilter.calculate(m_leftEncoder.getVelocity());
        var rightAverage = rightFilter.calculate(m_rightEncoder.getVelocity());

        SmartDashboard.putNumber("Shooter Average RPM (Left)", leftAverage);
        SmartDashboard.putNumber("Shooter Average RPM (Right)", rightAverage);

        if (setpoint != null) {
            if (leftAverage >= setpoint.getLeftVelocity() * Constants.ShooterConstants.autoShootRPMTolerance
                    && rightAverage >= setpoint.getRightVelocity() * Constants.ShooterConstants.autoShootRPMTolerance) {
                stateMachine.setState(State.Shoot);
            }
        }
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            // kicks automatically once piece is in place
            if (linebreak.get()) {
                kickerInstance.startKicking();
            }
        } // resets both shooter and kicker to idle
        if (!linebreak.get()) {
            stateMachine.setState(State.Idle);
            kickerInstance.startIdle();
        }
    }

    // button pressed by operator, pushes note out at slower speed
    private void handleJammedState(StateMetadata<State> metadata) {
        leftMotor.set(ShooterConstants.unjam);
        rightMotor.set(ShooterConstants.unjam);
    }

    private void handleReverseState(StateMetadata<State> metadata) {
        leftMotor.set(-0.05);
        rightMotor.set(-0.05);
    }

    public void setWarmUp(ShooterSetpoint setpoint) {
        this.setpoint = setpoint;
        stateMachine.setState(State.Warming);
    }

    public void setReverse() {
        stateMachine.setState(State.Reverse);
    }

    public void setJammed() {
        stateMachine.setState(State.Jammed);
    }

    public void reset() {
        stateMachine.setState(stateMachine.defaultStateKey);
    }

    @Override
    public void periodic() {
        stateMachine.update();

        // For tuning - should be replaced with ShuffleBoard tabs
        // double kp = SmartDashboard.getNumber("Shooter kP", 0);
        // double kd = SmartDashboard.getNumber("Shooter kD", 0);
        // double kf = SmartDashboard.getNumber("Shooter kF", 0);
        // double maxOutput = SmartDashboard.getNumber("Shooter max output", 1);
        // double setpointRPM = SmartDashboard.getNumber("Shooter setpoint (RPM)", 0);

        // var pidController = leftMotor.getPIDController();
        // pidController.setP(kp);
        // pidController.setD(kd);
        // pidController.setFF(kf);
        // pidController.setOutputRange(-maxOutput, maxOutput);
        // pidController.setReference(setpointRPM, ControlType.kVelocity);

        // pidController = rightMotor.getPIDController();
        // pidController.setP(kp);
        // pidController.setD(kd);
        // pidController.setFF(kf);
        // pidController.setOutputRange(-maxOutput, maxOutput);
        // pidController.setReference(setpointRPM, ControlType.kVelocity);

        // Log subsystem to AK
        Logger.recordOutput("Subsystems/Shooter/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Shooter/Has Note", !this.linebreak.get());
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }
}