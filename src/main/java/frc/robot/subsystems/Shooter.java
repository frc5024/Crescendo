package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.swing.text.AbstractDocument.LeafElement;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.controller.PIDController;
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
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;
    private PIDController leftPidController;
    private PIDController rightPidController;

    private boolean shootAmp;

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

        m_leftEncoder = leftMotor.getAlternateEncoder(Constants.Shooter.kCPR);
        m_rightEncoder = rightMotor.getAlternateEncoder(Constants.Shooter.kCPR);
        leftPidController = new PIDController(Constants.Shooter.pLeft, Constants.Shooter.iLeft, Constants.Shooter.dLeft);
        rightPidController = new PIDController(Constants.Shooter.pRight, Constants.Shooter.iRight, Constants.Shooter.dRight);
       
        leftPidController.setTolerance(Constants.Shooter.leftPidTolerance);
        rightPidController.setTolerance(Constants.Shooter.rightPidTolerance);

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
        if (metadata.isFirstRun()){
            leftPidController.reset();
            rightPidController.reset();

            if (shootAmp){

                leftPidController.setSetpoint(Constants.Shooter.shootSpeedAmp);
                rightPidController.setSetpoint(Constants.Shooter.shootSpeedAmp);
            
            }else{
               
                leftPidController.setSetpoint(Constants.Shooter.shootSpeedSpeaker);
                rightPidController.setSetpoint(Constants.Shooter.shootSpeedSpeaker);
            }
        }

        leftMotor.set(leftPidController.calculate(m_leftEncoder.getVelocity()));
        rightMotor.set(rightPidController.calculate(m_rightEncoder.getVelocity()));

        if (leftPidController.atSetpoint() && rightPidController.atSetpoint()){
            stateMachine.setState(State.Shoot);
        }

    }

    private void handleShootState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            shootTimer.reset();
            shootTimer.start();
            kicker.set(-0.3); //for testing purposes
        }

        if (shootTimer.hasElapsed(1.5)) {
            stateMachine.setState(State.Idle);
            shootTimer.stop();
        }
    }

    public void setWarmUp(){
        stateMachine.setState(State.Warming);
    }

    @Override
    public void periodic() {
        stateMachine.update();
    }
}
