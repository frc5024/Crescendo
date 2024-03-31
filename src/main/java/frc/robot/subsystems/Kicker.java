package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

    // statics
    private static Kicker mInstance = null;

    // Singleton
    public static Kicker getInstance() {
        if (mInstance == null) {
            mInstance = new Kicker();
        }

        return mInstance;
    }

    // non statics
    private CANSparkMax kickerMotor;
    private Timer pullbackTimer;

    // States
    public enum State {
        Idle,
        Kicking,
        Pullback,
        Intaking,
        Jammed,
        Pushing,
        Shooting,
    }

    protected StateMachine<State> stateMachine;

    private Kicker() {
        // sets up state machine
        stateMachine = new StateMachine<>("Kicker");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Kicking, this::handleKickingState);
        stateMachine.addState(State.Pullback, this::handlePullbackState);
        stateMachine.addState(State.Intaking, this::handleIntakingState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
        stateMachine.addState(State.Pushing, this::handlePushingState);
        stateMachine.addState(State.Shooting, this::handleShootingState);

        // initializes components
        kickerMotor = new CANSparkMax(Constants.KickerConstants.kickerMotor, MotorType.kBrushless);
        kickerMotor.setInverted(true);
        pullbackTimer = new Timer();
        var Tab = Shuffleboard.getTab("Test");
        Tab.addDouble("Pullback timer", () -> pullbackTimer.get());

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

    // intake and kicker overshoot the intaking, this pulls the note back for a set
    // period of time
    private void handlePullbackState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            pullbackTimer.reset();
            pullbackTimer.start();
            kickerMotor.set(Constants.KickerConstants.kickerPullbackSpeed);
        }
        if (pullbackTimer.get() > Constants.KickerConstants.pullbackTimer) {
            pullbackTimer.stop();
            stateMachine.setState(State.Idle);
        }
    }

    private void handleIntakingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            kickerMotor.set(Constants.KickerConstants.kickerIntakingSpeed);
        }
    }

    private void handleJammedState(StateMetadata<State> metadata) {
        kickerMotor.set(Constants.ShooterConstants.unjam);
    }

    private void handlePushingState(StateMetadata<State> metadata) {
        kickerMotor.set(Constants.ShooterConstants.intake);
    }

    private void handleShootingState(StateMetadata<State> metadata) {
        kickerMotor.set(Constants.KickerConstants.kickerShootSpeed);
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

    public void startJammed() {
        stateMachine.setState(State.Jammed);
    }

    public void startPushing() {
        stateMachine.setState(State.Pushing);
    }

    public void startShooting() {
        stateMachine.setState(State.Shooting);
    }

    public void reset() {
        stateMachine.setState(stateMachine.defaultStateKey);
    }

    // Getters
    public State getCurrentState() {
        return stateMachine.getCurrentState();
    }

    @Override
    public void periodic() {
        stateMachine.update();

        // Log subsystem to AK
        Logger.recordOutput("Subsystems/Kicker/CurrentState", getCurrentState());
        Logger.recordOutput("Subsystems/Kicker/AppliedOutput", kickerMotor.getAppliedOutput());
        Logger.recordOutput("Subsystems/Kicker/Voltage", kickerMotor.getBusVoltage());
    }
}
