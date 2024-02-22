package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    private Kicker kickerInstance;
    private DigitalInput linebreak;

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

        kickerInstance = Kicker.getInstance();

        var Tab = Shuffleboard.getTab("Test");
        Tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        Tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        Tab.addBoolean("Linebroken", () -> linebreak.get());

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            leftMotor.set(0.0);
            rightMotor.set(0.0);
        }
    }

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            leftMotor.set(1);
            rightMotor.set(1);
        }

        if (m_leftEncoder.getVelocity() >= Constants.ShooterConstants.speakerSetpoint
                && m_rightEncoder.getVelocity() >= Constants.ShooterConstants.speakerSetpoint) {
            stateMachine.setState(State.Shoot);
        }
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            if (linebreak.get()) {
                kickerInstance.startKicking();
            }
        }
        if (!linebreak.get()) {
            stateMachine.setState(State.Idle);
            kickerInstance.startIdle();
        }
    }

    private void handleJammedState(StateMetadata<State> metadata) {
        leftMotor.set(ShooterConstants.unjam);
        rightMotor.set(ShooterConstants.unjam);
    }

    public void setWarmUp() {
        stateMachine.setState(State.Warming);
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

        // Log subsystem to AK
        Logger.recordOutput("Subsystems/Shooter/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Shooter/Has Note", !this.linebreak.get());
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }
}