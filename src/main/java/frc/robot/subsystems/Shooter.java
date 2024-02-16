package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance = null;

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    // private CANSparkMax kicker;

    private Timer warmingUp;
    private Timer shootTimer;
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    private Kicker kickerInstance;
    private DigitalInput linebreak;
    private PIDController leftPIDController;
    private PIDController rightPIDController;

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
        // kicker = new CANSparkMax(60, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        // kicker.restoreFactoryDefaults();

        leftMotor.setInverted(true);

        m_leftEncoder = leftMotor.getEncoder();
        m_rightEncoder = rightMotor.getEncoder();
        leftPIDController = new PIDController(0.2, 0.2, 0);
        rightPIDController = new PIDController(0.2, 0.2, 0);

        leftMotor.setSmartCurrentLimit(ShooterConstants.ShooterCurrentLimit);
        rightMotor.setSmartCurrentLimit(ShooterConstants.ShooterCurrentLimit);

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
        SmartDashboard.putNumber("Shooter kP", 0.000050);
        SmartDashboard.putNumber("Shooter max output", 0);
        SmartDashboard.putNumber("Shooter setpoint (RPM)", 0);
        SmartDashboard.putNumber("Shooter accelerator RPM", 0);

        stateMachine = new StateMachine<>("Shooter");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        stateMachine.addState(State.Shoot, this::handleShootState);
        stateMachine.addState(State.Jammed, this::handleJammedState);

    }

    // public void setFlywheelToRPM(double rpm) {
    // leftMotor.getPIDController().setReference(rpm,
    // CANSparkBase.ControlType.kVelocity);
    // rightMotor.getPIDController().setReference(rpm,
    // CANSparkBase.ControlType.kVelocity);
    // }

    private void handleIdleState(StateMetadata<State> metadata) {

        if (metadata.isFirstRun()) {
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            // kickerInstance.startIdle();
        }

    }

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            warmingUp.reset();
            warmingUp.start();
            leftPIDController.reset();
            rightPIDController.reset();
            leftPIDController.setTolerance(Constants.ShooterConstants.tolerance,
                    Constants.ShooterConstants.errorDerivative);
            rightPIDController.setTolerance(Constants.ShooterConstants.tolerance,
                    Constants.ShooterConstants.errorDerivative);

            // leftMotor.set(0.9);
            // rightMotor.set(0.9);
        }

        if (Constants.ShooterConstants.speaker = true) {
            leftMotor.set(leftPIDController.calculate(m_leftEncoder.getVelocity(),
                    Constants.ShooterConstants.speakerSetpoint));
            rightMotor.set(rightPIDController.calculate(m_rightEncoder.getVelocity(),
                    Constants.ShooterConstants.speakerSetpoint));
        } else {
            leftMotor.set(
                    leftPIDController.calculate(m_leftEncoder.getVelocity(), Constants.ShooterConstants.ampSetpoint));
            rightMotor.set(
                    rightPIDController.calculate(m_rightEncoder.getVelocity(), Constants.ShooterConstants.ampSetpoint));
        }

        if (leftPIDController.atSetpoint() && rightPIDController.atSetpoint()) {
            warmingUp.stop();
            stateMachine.setState(State.Shoot);
        }

        // if (m_leftEncoder.getVelocity() >=
        // Constants.ShooterConstants.shootVelocitySpeaker
        // && m_rightEncoder.getVelocity() >=
        // Constants.ShooterConstants.shootVelocitySpeaker) {
        // warmingUp.stop();
        // stateMachine.setState(State.Shoot);
        // }
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

    @Override
    public void periodic() {
        stateMachine.update();

        // double kp = SmartDashboard.getNumber("Shooter kP", 0);
        // double maxOutput = SmartDashboard.getNumber("Shooter max output", 0);
        // double setpointRPM = SmartDashboard.getNumber("Shooter setpoint (RPM)", 0);

        // SparkPIDController leftPIDController = leftMotor.getPIDController();
        // SparkPIDController rightPIDController = rightMotor.getPIDController();

        // leftPIDController.setP(kp);
        // leftPIDController.setOutputRange(-maxOutput, maxOutput);
        // leftPIDController.setReference(setpointRPM,
        // CANSparkBase.ControlType.kVelocity);
        // rightPIDController.setP(kp);
        // rightPIDController.setOutputRange(-maxOutput, maxOutput);
        // rightPIDController.setReference(setpointRPM,
        // CANSparkBase.ControlType.kVelocity);

        SmartDashboard.putNumber("Left flywheel velocity",
                m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right flywheel velocity",
                m_rightEncoder.getVelocity());

        // SmartDashboard.putNumber("Left flywheel error", m_leftEncoder.getVelocity() -
        // setpointRPM);
        // SmartDashboard.putNumber("Right flywheel error", m_rightEncoder.getVelocity()
        // - setpointRPM);
    }

    public boolean isLineBroken() {
        return linebreak.get();
    }
}
