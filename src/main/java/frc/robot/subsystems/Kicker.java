package frc.robot.subsystems;

import io.github.frc5024.libkontrol.StateMachine;
import io.github.frc5024.libkontrol.StateMetadata;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
    private static Kicker mInstance = null;
    private Talon kickerMotor;

    public static Kicker getInstance() {
        if (mInstance == null) {
            mInstance = new Kicker();
        }

        return mInstance;
    }

    public enum State {
        Idle,
        Kicking,
        Holding,
        Intaking
    }

    protected StateMachine<State> stateMachine;

    private Kicker() {
        stateMachine = new StateMachine<>("Kicker");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Kicking, this::handleKickingState);
        stateMachine.addState(State.Holding, this::handleHoldingState);
        stateMachine.addState(State.Intaking, this::handleIntakingState);
        kickerMotor = new Talon(Constants.KickerConstants.kickerMotor);
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        kickerMotor.set(0);

    }

    private void handleKickingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            kickerMotor.set(Constants.KickerConstants.kickerSpeed);
        }
    }

    private void handleHoldingState(StateMetadata<State> metadata) {

    }

    private void handleIntakingState(StateMetadata<State> metadata) {

    }

    public void startKicking() {
        stateMachine.setState(State.Kicking);
    }

    public void startIdle() {
        stateMachine.setState(State.Idle);
    }

    public void startHolding() {
        stateMachine.setState(State.Holding);
    }

    public void startIntaking() {
        stateMachine.setState(State.Intaking);
    }

    public State getCurrentState() {
        return stateMachine.getCurrentState();
    }
}
