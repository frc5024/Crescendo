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
import edu.wpi.first.wpilibj.Timer;
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

    private ShooterSetpoint currentSetpoint = ShooterSetpoint.speakerSetpoint;
    private ShooterSetpoint desiredSetpoint = ShooterSetpoint.speakerSetpoint;

    private double rightAverage;
    private double leftAverage;

    private Timer jammed;

    // LinearFilters to calculate the average RPM of the encoders
    private LinearFilter leftFilter = LinearFilter.movingAverage(Constants.ShooterConstants.autoShootSampleCount);
    private LinearFilter rightFilter = LinearFilter.movingAverage(Constants.ShooterConstants.autoShootSampleCount);

    private Timer intake;
    private Timer trapShootDelay;

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
        Reverse,
        Trap
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

        jammed = new Timer();

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

        intake = new Timer();
        trapShootDelay = new Timer();

        // shuffleboard tabs: velocity for both encoders and the linebreak
        var tab = Shuffleboard.getTab("Shooter");
        tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        tab.addBoolean("Linebroken", () -> linebreak.get());
        tab.addBoolean("WarmedUp", () -> warmedUp());

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
        stateMachine.addState(State.Reverse, this::handleReverseState);
        stateMachine.addState(State.Trap, this::handleTrapState);

        SmartDashboard.putNumber("Shooter kP", 0.000012);
        SmartDashboard.putNumber("Shooter kD", 0.002);
        SmartDashboard.putNumber("Shooter kF", 0.000172);
        SmartDashboard.putNumber("Shooter max output", 1);
        SmartDashboard.putNumber("Shooter setpoint (RPM)", 0);

        SmartDashboard.putNumber("Shooter Average RPM (Left)", 0);
        SmartDashboard.putNumber("Shooter Average RPM (Right)", 0);
    }

    public void setTargetVelocity(ShooterSetpoint setpoint) {
        if (setpoint == null)
            setpoint = ShooterSetpoint.zero;

        currentSetpoint = setpoint;

        leftMotor.getPIDController().setReference(setpoint.getLeftVelocity(), ControlType.kVelocity);
        rightMotor.getPIDController().setReference(setpoint.getRightVelocity(), ControlType.kVelocity);

        SmartDashboard.putString("Shooter Setpoint", setpoint.name());
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            setTargetVelocity(ShooterSetpoint.zero);
        }
    }

    private boolean triedPullback = false;
    private Timer attemptPullbackTimer = new Timer();

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            setTargetVelocity(desiredSetpoint);
            leftFilter.reset();
            rightFilter.reset();
            triedPullback = false;
            attemptPullbackTimer.reset();
            attemptPullbackTimer.start();
        }

        leftAverage = leftFilter.calculate(m_leftEncoder.getVelocity());
        rightAverage = rightFilter.calculate(m_rightEncoder.getVelocity());

        // If we're trying to warm up, but the velocity isn't increasing, a note is
        // likely jammed with a note, so run the Kicker pullback to unjam it
        if (!triedPullback && attemptPullbackTimer.hasElapsed(ShooterConstants.warmupJamPullbackTime)
                && (leftAverage < ShooterConstants.warmupJamVelocityThreshold
                        || rightAverage < ShooterConstants.warmupJamVelocityThreshold)) {
            triedPullback = true;
            Kicker.getInstance().startPullback();
        }

        SmartDashboard.putNumber("Shooter Average RPM (Left)", leftAverage);
        SmartDashboard.putNumber("Shooter Average RPM (Right)", rightAverage);
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            // kicks automatically once piece is in place
            leftFilter.reset();
            rightFilter.reset();
            if (linebreak.get()) {
                kickerInstance.startKicking();
            }
        }
        // resets both shooter and kicker to idle
        if (!linebreak.get()) {
            stateMachine.setState(State.Idle);
            kickerInstance.startIdle();
        }
    }

    // button pressed by operator, pushes note out at slower speed
    private void handleJammedState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            jammed.reset();
            jammed.start();
        }

        leftMotor.set(ShooterConstants.unjam);
        rightMotor.set(ShooterConstants.unjam);

        if (jammed.hasElapsed(0.5)) {
            kickerInstance.startJammed();
            jammed.stop();
        }
        if (!linebreak.get()) {
            kickerInstance.startIdle();
            reset();
        }
    }

    private void handleReverseState(StateMetadata<State> metadata) {
        if (!linebreak.get()) {
            stateMachine.setState(State.Idle);
            kickerInstance.startIdle();
        } else {
            leftMotor.set(-0.05);
            rightMotor.set(-0.05);
        }
    }

    private void handleTrapState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()){
            intake.reset();
            trapShootDelay.reset();
            intake.start();
        }

        // kicker and shooter motors push the piece back into the shooter wheels
        leftMotor.set(Constants.ShooterConstants.intake);
        rightMotor.set(Constants.ShooterConstants.intake);
        kickerInstance.startPushing();

        if (intake.hasElapsed(0.25)) {
            intake.stop();

            leftMotor.set(0);
            rightMotor.set(0);

            kickerInstance.startShooting();
            //warmup timer
            trapShootDelay.start();

            if (trapShootDelay.hasElapsed(1.0)) {
                trapShootDelay.stop();
                stateMachine.setState(State.Reverse);
            }
        }
    }

    public void setWarmUp() {
        stateMachine.setState(State.Warming);
    }

    public void setShoot() {
        stateMachine.setState(State.Shoot);
    }

    public void setReverse() {
        stateMachine.setState(State.Reverse);
    }

    public void setJammed() {
        stateMachine.setState(State.Jammed);
    }

    public void setTrap() {
        stateMachine.setState(State.Trap);
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
        Logger.recordOutput("Subsystems/Shooter/CurrentState", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Shooter/HasNote", this.linebreak.get());
        Logger.recordOutput("Subsystems/Shooter/AppliedOutput", leftMotor.getAppliedOutput());
        Logger.recordOutput("Subsystems/Shooter/LeftVelocity", m_leftEncoder.getVelocity());
        Logger.recordOutput("Subsystems/Shooter/RightVelocity", m_rightEncoder.getVelocity());
        Logger.recordOutput("Subsystems/Shooter/LeftAverage", leftFilter.calculate(m_leftEncoder.getVelocity()));
        Logger.recordOutput("Subsystems/Shooter/RightAverage", rightFilter.calculate(m_rightEncoder.getVelocity()));
        Logger.recordOutput("Subsystems/Shooter/Voltage", leftMotor.getBusVoltage());
    }

    public void setDesiredSetpoint(ShooterSetpoint setpoint) {
        // desiredSetpoint will be applied when warming up
        this.desiredSetpoint = setpoint;

        // if we're warming up and the setpoint changed, update it
        if (stateMachine.getCurrentState() == State.Warming) {
            setTargetVelocity(setpoint);
            leftFilter.reset();
            rightFilter.reset();
        }
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    public boolean warmedUp() {
        if (currentSetpoint != null && currentSetpoint != ShooterSetpoint.zero) {
            if (currentSetpoint.getLeftVelocity() <= 2500) {
                if (leftAverage >= currentSetpoint.getLeftVelocity() * 0.85
                        && rightAverage >= currentSetpoint.getRightVelocity() * 0.85) {
                    return true;
                }
            }
            if (leftAverage >= currentSetpoint.getLeftVelocity() * ShooterConstants.autoShootRPMTolerance
                    && rightAverage >= currentSetpoint.getRightVelocity() * ShooterConstants.autoShootRPMTolerance) {
                return true;
            }
        }
        return false;
    }
}