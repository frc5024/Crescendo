package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax kicker;

    private Timer warmingUp;
    private Timer shootTimer;

    public static Shooter getInstance() {
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
        Jostle,
        Expel,
    }

    protected StateMachine<State> stateMachine;

    private Shooter() {
        leftMotor = new CANSparkMax(Constants.Shooter.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.Shooter.rightMotorId, MotorType.kBrushless);
        kicker = new CANSparkMax(60, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        kicker.restoreFactoryDefaults();

        warmingUp = new Timer();
        warmingUp.reset();
        shootTimer = new Timer();

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);

    }

    private void handleIdleState(StateMetadata<State> metadata) {

        if (metadata.isFirstRun()) {
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            kicker.set(0.0);
        }

    }

    private void handleWarmingState(StateMetadata<State> metadata) {

        if (metadata.isFirstRun()) {
            warmingUp.reset();
            leftMotor.set(0.8);
            rightMotor.set(0.8);
            warmingUp.start();
        }

        if (warmingUp.hasElapsed(1.5)) {
            stateMachine.setState(State.Shoot);
            warmingUp.stop();
        }

    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            shootTimer.reset();
            shootTimer.start();
            kicker.set(0.8);
        }

        if (shootTimer.hasElapsed(1.5)) {
            stateMachine.setState(State.Idle);
            shootTimer.stop();
        }
    }

    public void setWarmUp() {
        System.out.println("YTESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
        stateMachine.setState(State.Warming);
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
