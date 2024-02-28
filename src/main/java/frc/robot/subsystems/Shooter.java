package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.dashboard.SmarterDashboard;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // LinearFilters to calculate the average RPM of the encoders
    private LinearFilter leftFilter = LinearFilter.movingAverage(ShooterConstants.autoShootSampleCount);
    private LinearFilter rightFilter = LinearFilter.movingAverage(ShooterConstants.autoShootSampleCount);

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

        leftMotor = new CANSparkMax(ShooterConstants.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ShooterConstants.rightMotorId, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();

        leftMotor.getEncoder().setVelocityConversionFactor(1.0);
        leftMotor.getEncoder().setPositionConversionFactor(1.0 / ShooterConstants.gearRatio);

        rightMotor.getEncoder().setVelocityConversionFactor(1.0);
        rightMotor.getEncoder().setPositionConversionFactor(1.0 / ShooterConstants.gearRatio);

        leftMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);
        rightMotor.setSmartCurrentLimit(ShooterConstants.currentLimitAmps);

        var pidController = leftMotor.getPIDController();
        pidController.setP(ShooterConstants.kP);
        pidController.setD(ShooterConstants.kD);
        pidController.setFF(ShooterConstants.kF);
        pidController.setOutputRange(-ShooterConstants.maxOutput, ShooterConstants.maxOutput);

        pidController = rightMotor.getPIDController();
        pidController.setP(ShooterConstants.kP);
        pidController.setD(ShooterConstants.kD);
        pidController.setFF(ShooterConstants.kF);
        pidController.setOutputRange(-ShooterConstants.maxOutput, ShooterConstants.maxOutput);

        kickerInstance = Kicker.getInstance();

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
        stateMachine.addState(State.Reverse, this::handleReverseState);
    }

    public void setTargetVelocity(ShooterSetpoint setpoint) {
        if (setpoint == null)
            setpoint = ShooterSetpoint.zero;

        currentSetpoint = setpoint;

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
            setTargetVelocity(desiredSetpoint);
            leftFilter.reset();
            rightFilter.reset();
        }

        leftAverage = leftFilter.calculate(m_leftEncoder.getVelocity());
        rightAverage = rightFilter.calculate(m_rightEncoder.getVelocity());

        SmarterDashboard.putNumber("Shooter/Left Motor/Average RPM", leftAverage);
        SmarterDashboard.putNumber("Shooter/Right Motor/Average RPM", rightAverage);
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            // kicks automatically once piece is in place
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
        leftMotor.set(ShooterConstants.unjam);
        rightMotor.set(ShooterConstants.unjam);
    }

    private void handleReverseState(StateMetadata<State> metadata) {
        leftMotor.set(-0.05);
        rightMotor.set(-0.05);
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

    public void reset() {
        stateMachine.setState(stateMachine.defaultStateKey);
    }

    @Override
    public void periodic() {
        stateMachine.update();

        SmarterDashboard.putNumber("Shooter/Left Motor/Velocity", m_leftEncoder.getVelocity());
        SmarterDashboard.putNumber("Shooter/Right Motor/Velocity", m_rightEncoder.getVelocity());

        SmarterDashboard.putBoolean("Shooter/Linebreak", isLineBroken());
        SmarterDashboard.putBoolean("Shooter/WarmedUp", warmedUp());

        SmarterDashboard.putString("Shooter/Setpoint", currentSetpoint == null ? "None" : currentSetpoint.name());

        double kP = SmarterDashboard.getNumber("Shooter/kP", ShooterConstants.kP);
        double kD = SmarterDashboard.getNumber("Shooter/kD", ShooterConstants.kD);
        double kF = SmarterDashboard.getNumber("Shooter/kF", ShooterConstants.kF);
        double maxOutput = SmarterDashboard.getNumber("Shooter/Max Output", ShooterConstants.maxOutput);

        if (SmarterDashboard.getBoolean("Shooter/Debug", false)) {
            double debugSetpoint = SmarterDashboard.getNumber("Shooter/Debug/Setpoint", 0);

            var pidController = leftMotor.getPIDController();
            pidController.setP(kP);
            pidController.setD(kD);
            pidController.setFF(kF);
            pidController.setOutputRange(-maxOutput, maxOutput);
            pidController.setReference(debugSetpoint, ControlType.kVelocity);

            pidController = rightMotor.getPIDController();
            pidController.setP(kP);
            pidController.setD(kD);
            pidController.setFF(kF);
            pidController.setOutputRange(-maxOutput, maxOutput);
            pidController.setReference(debugSetpoint, ControlType.kVelocity);
        }

        SmarterDashboard.putString("StateMachine/" + stateMachine.getName(), stateMachine.getCurrentState().toString());

        // Log subsystem to AK
        Logger.recordOutput("Subsystems/Shooter/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Shooter/Has Note", !this.linebreak.get());
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
        if (currentSetpoint != null) {
            if (leftAverage >= currentSetpoint.getLeftVelocity() * ShooterConstants.autoShootRPMTolerance
                    && rightAverage >= currentSetpoint.getRightVelocity() * ShooterConstants.autoShootRPMTolerance) {
                return true;
            }
        }
        return false;
    }
}