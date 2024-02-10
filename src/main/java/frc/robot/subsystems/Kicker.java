package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

    private static Kicker mInstance = null;
    private CANSparkMax kickerMotor;

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
        kickerMotor = new CANSparkMax(Constants.KickerConstants.kickerMotor, MotorType.kBrushless);
        kickerMotor.setInverted(true);
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        // Stops motors
        if (metadata.isFirstRun()) {
            kickerMotor.set(0);
        }

    }

    private void handleKickingState(StateMetadata<State> metadata) {
        // sets motors to kicker speed
        // if (metadata.isFirstRun()) {
        kickerMotor.set(Constants.KickerConstants.kickerSpeed);
        // }
    }

    private void handleHoldingState(StateMetadata<State> metadata) {
        // Waiting to be done, will hold the piece when linebreak sensor in shooter is
        // broken to hold piece until ready to shoot
    }

    private void handleIntakingState(StateMetadata<State> metadata) {
        // Waiting to be done, will activate motors when intake button is pressed to
        // take in piece from intake to kicker
    }

    // Setters
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

    // Getters
    public State getCurrentState() {
        return stateMachine.getCurrentState();
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
