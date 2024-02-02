package frc.robot.subsystems;

import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake mInstance = null;
  private Talon topRoller;

  // private Talon bottomRoller;
  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }

    return mInstance;
  }

  public enum State {
    Idle,
    Intaking,
    ReverseIntake
  }

  protected StateMachine<State> stateMachine;

  private Intake() {
    stateMachine = new StateMachine<>("Intake");
    stateMachine.setDefaultState(State.Idle, this::handleIdleState);
    stateMachine.addState(State.Intaking, this::handleIntakingState);
    stateMachine.addState(State.ReverseIntake, this::handleReverseIntakeState);
    topRoller = new Talon(Constants.IntakeConstants.topRollerChanel);
    // bottomRoller = new Talon(Constants.Intake.bottomRollerChanel);
  }

  private void handleIdleState(StateMetadata<State> metadata) {
    topRoller.set(0);
    // bottomRoller.set(0);
  }

  private void handleIntakingState(StateMetadata<State> metadata) {
    if (metadata.isFirstRun()) {
      topRoller.set(Constants.IntakeConstants.rollerSpeed);
    }

  }

  private void handleReverseIntakeState(StateMetadata<State> metadata) {
    if (metadata.isFirstRun()) {
      topRoller.set(Constants.IntakeConstants.reverseRollerSpeed);
    }
  }

  public void startIntaking() {
    stateMachine.setState(State.Intaking);
  }

  public void startIdle() {
    stateMachine.setState(State.Idle);
  }

  public void startReverseIntake() {
    stateMachine.setState(State.ReverseIntake);
  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }

  @Override
  public void periodic() {
    stateMachine.update();
  }
}
