package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

    private static Kicker mInstance = null;
    private CANSparkMax kickerMotor;
    private Timer pullback;

    // Singleton
    public static Kicker getInstance() {
        if (mInstance == null) {
            mInstance = new Kicker();
        }

        return mInstance;
    }

    // States
    public enum State {
        Idle,
        Kicking,
        Pullback,
        Intaking
    }

    protected StateMachine<State> stateMachine;

    private Kicker() {
        stateMachine = new StateMachine<>("Kicker");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Kicking, this::handleKickingState);
        stateMachine.addState(State.Pullback, this::handlePullbackState);
        stateMachine.addState(State.Intaking, this::handleIntakingState);
        kickerMotor = new CANSparkMax(Constants.KickerConstants.kickerMotor, MotorType.kBrushless);
        kickerMotor.setInverted(true);
        pullback = new Timer();
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        // Stops motors
        if (metadata.isFirstRun()) {
            kickerMotor.set(0);
        }

    }

    private void handleKickingState(StateMetadata<State> metadata) {
        // sets motors to kicker speed
        if (metadata.isFirstRun()) {
            kickerMotor.set(Constants.KickerConstants.kickerSpeed);
        }
    }

    private void handlePullbackState(StateMetadata<State> metadata) {
        pullback.reset();
        pullback.start();
        kickerMotor.set(Constants.KickerConstants.kickerPullbackSpeed);
        if (pullback.get() > Constants.KickerConstants.pullbackTimer) {
            pullback.stop();
            stateMachine.setState(State.Idle);
        }
    }

    private void handleIntakingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            kickerMotor.set(Constants.KickerConstants.kickerIntakingSpeed);
        }
        if (Shooter.getInstance().isLineBroken()) {
            stateMachine.setState(State.Pullback);
        }
    }

    // Setters
    public void startKicking() {
        stateMachine.setState(State.Kicking);
    }

    public void startIdle() {
        stateMachine.setState(State.Idle);
    }

    public void startPullback() {
        stateMachine.setState(State.Pullback);
    }

    public void startIntaking() {
        stateMachine.setState(State.Intaking);
    }

    // Getters
    public State getCurrentState() {
        return stateMachine.getCurrentState();
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
