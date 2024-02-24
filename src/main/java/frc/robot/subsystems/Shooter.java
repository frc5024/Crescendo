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
import frc.robot.Constants.ShooterConstants.ShooterSetpoint;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    private Kicker kickerInstance;
    private DigitalInput linebreak;

    private ShooterSetpoint setpoint;

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

        leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorId, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorId, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();

        kickerInstance = Kicker.getInstance();

        // shuffleboard tabs: velocity for both encoders and the linebreak
        var Tab = Shuffleboard.getTab("Test");
        Tab.addDouble("LeftEncoderVelocity", () -> m_leftEncoder.getVelocity());
        Tab.addDouble("RightEncoderVelocity", () -> m_rightEncoder.getVelocity());
        Tab.addBoolean("Linebroken", () -> linebreak.get());

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);
        stateMachine.addState(State.Reverse, this::handleReverseState);
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

        // shoots once the motors get to the set speed
        if (setpoint != null && m_leftEncoder.getVelocity() >= setpoint.getLeftVelocity()
                && m_rightEncoder.getVelocity() >= setpoint.getRightVelocity()) {
            stateMachine.setState(State.Shoot);
        }
    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            // kicks automatically once piece is in place
            if (linebreak.get()) {
                kickerInstance.startKicking();
            }
        } // resets both shooter and kicker to idle
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

        // Log subsystem to AK
        Logger.recordOutput("Subsystems/Shooter/Current State", this.stateMachine.getCurrentState());
        Logger.recordOutput("Subsystems/Shooter/Has Note", !this.linebreak.get());
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }

    public void setSetpoint(ShooterSetpoint setpoint) {
        this.setpoint = setpoint;
    }
}