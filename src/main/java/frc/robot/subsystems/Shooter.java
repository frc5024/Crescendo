package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    //private CANSparkMax kicker;

    private Timer warmingUp;
    private Timer shootTimer;
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
    }

    protected StateMachine<State> stateMachine;

    private Shooter() {
        linebreak = new DigitalInput(8);

        leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorId, MotorType.kBrushless);
        // kicker = new CANSparkMax(60, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        // kicker.restoreFactoryDefaults();

        leftMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();

        warmingUp = new Timer();
        warmingUp.reset();
        shootTimer = new Timer();

        kickerInstance = Kicker.getInstance();

        var Tab = Shuffleboard.getTab("Test");
        Tab.addDouble("LeftEncoder", () -> m_leftEncoder.getPosition());
        Tab.addDouble("RightEncoder", () -> m_rightEncoder.getPosition());
        Tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        Tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        Tab.addDouble("warming timer", () -> warmingUp.get());
        Tab.addDouble("shootTimer", () -> shootTimer.get());
        Tab.addBoolean("Linebroken", () -> linebreak.get());
    

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);

    }

    private void handleIdleState(StateMetadata<State> metadata) {

        if (metadata.isFirstRun()) {
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            //kickerInstance.startIdle();
        }

    }

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            warmingUp.reset();
            warmingUp.start();
            leftMotor.set(0.9);
            rightMotor.set(0.9);
        }

        if (m_leftEncoder.getVelocity() >= Constants.ShooterConstants.shootVelocitySpeaker
                && m_rightEncoder.getVelocity() >= Constants.ShooterConstants.shootVelocitySpeaker) {
            warmingUp.stop();
            stateMachine.setState(State.Shoot);
        }
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            if (linebreak.get()) {
                shootTimer.reset();
                shootTimer.start();
                kickerInstance.startKicking();
                // kicker.set(-0.8);// for testing purposes
            }
        }
        if (!linebreak.get()) {
            shootTimer.stop();
            stateMachine.setState(State.Idle);
            kickerInstance.startIdle();
        }
    }

    public void setWarmUp() {
        stateMachine.setState(State.Warming);
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }
}
