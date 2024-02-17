package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  // statics
  private static Intake mInstance = null;

  // singleton
  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }

    return mInstance;
  }

  // non statics
  private TalonSRX topRoller;

  // states
  public enum State {
    Idle,
    Intaking,
    Outtake
  }

  protected StateMachine<State> stateMachine;

  private Intake() {
    // setting up states
    stateMachine = new StateMachine<>("Intake");
    stateMachine.setDefaultState(State.Idle, this::handleIdleState);
    stateMachine.addState(State.Intaking, this::handleIntakingState);
    stateMachine.addState(State.Outtake, this::handleOuttake);

    // initializing physical components
    topRoller = new TalonSRX(Constants.IntakeConstants.topRollerChannel);
    // Invert rollers so positive is forward
    topRoller.setInverted(true);
  }

  // methods for handling states
  private void handleIdleState(StateMetadata<State> metadata) {
    // stops motors
    if (metadata.isFirstRun()) {
      // sets motors speed to zero
      topRoller.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  private void handleIntakingState(StateMetadata<State> metadata) {
    // makes sure motors don't try to set speed repeatedly
    if (metadata.isFirstRun()) {
      // sets motors speed
      topRoller.set(TalonSRXControlMode.PercentOutput, Constants.IntakeConstants.intakeSpeed);
    }
  }

  // used for ejecting jammed pieces
  private void handleOuttake(StateMetadata<State> metadata) {
    // makes sure motots don't try to set speed repeatedly
    if (metadata.isFirstRun()) {
      // sets motors speed
      topRoller.set(TalonSRXControlMode.PercentOutput, Constants.IntakeConstants.outtakeSpeed);
    }
  }

  // Setters
  public void startIntaking() {
    stateMachine.setState(State.Intaking);
  }

  public void startIdle() {
    stateMachine.setState(State.Idle);
  }

  public void startOuttake() {
    stateMachine.setState(State.Outtake);
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
