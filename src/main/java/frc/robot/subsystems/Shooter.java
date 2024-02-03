package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax kicker;

    private Timer warmingUp;
    private Timer shootTimer;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

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
    }

    protected StateMachine<State> stateMachine;

    private Shooter() {
        leftMotor = new CANSparkMax(Constants.Shooter.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.Shooter.rightMotorId, MotorType.kBrushless);
        kicker = new CANSparkMax(60, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        kicker.restoreFactoryDefaults();

       rightMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();

        warmingUp = new Timer();
        warmingUp.reset();
        shootTimer = new Timer();
     
        var Tab = Shuffleboard.getTab("Test");
        Tab.addDouble("LeftEncoder", () -> m_leftEncoder.getPosition());
        Tab.addDouble("RightEncoder", () -> m_rightEncoder.getPosition());
        Tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        Tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        Tab.addDouble("warming timer", () -> warmingUp.get());
        Tab.addDouble("shootTimer", () -> shootTimer.get());

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
            warmingUp.start();
            leftMotor.set(0.9);
            rightMotor.set(0.9);
        }

        if (m_leftEncoder.getVelocity() >= Constants.Shooter.shootVelocitySpeaker && m_rightEncoder.getVelocity() >= Constants.Shooter.shootVelocitySpeaker){
            warmingUp.stop();
            stateMachine.setState(State.Shoot);
        } 
    }
    

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            shootTimer.reset();
            shootTimer.start();
            kicker.set(-0.8);// for testing purposes

        }

        if (shootTimer.hasElapsed(1.5)) {
            stateMachine.setState(State.Idle);
            shootTimer.stop();
        }
    }

    public void setWarmUp() {
        stateMachine.setState(State.Warming);
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
