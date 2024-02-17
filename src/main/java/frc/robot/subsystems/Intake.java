package frc.robot.subsystems;

import io.github.frc5024.libkontrol.StateMachine;
import io.github.frc5024.libkontrol.StateMetadata;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake mInstance = null;
  private Talon topRoller; // move this where your declaring state machine

  // singleton
  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }

    return mInstance;
  }

  // states
  public enum State {
    Idle,
    Intaking,
    Outtake
  }

  protected StateMachine<State> stateMachine;

  private Intake() {
    stateMachine = new StateMachine<>("Intake");
    stateMachine.setDefaultState(State.Idle, this::handleIdleState);
    stateMachine.addState(State.Intaking, this::handleIntakingState);
    stateMachine.addState(State.Outtake, this::handleOuttake);

    topRoller = new Talon(Constants.IntakeConstants.topRollerChanel);
    // Invert rollers so positive is forward
    topRoller.setInverted(true);
  }

  private void handleIdleState(StateMetadata<State> metadata) {
    // stops motors
    if (metadata.isFirstRun()) {
      topRoller.set(0);
    }
  }

  private void handleIntakingState(StateMetadata<State> metadata) {
    // makes sure motors don't try to set speed repeatedly
    if (metadata.isFirstRun()) {
      topRoller.set(Constants.IntakeConstants.intakeSpeed);
    }
  }

  // used for ejecting jammed pieces
  private void handleOuttake(StateMetadata<State> metadata) {
    // makes sure motots don't try to set speed repeatedly
    if (metadata.isFirstRun()) {
      topRoller.set(Constants.IntakeConstants.outtakeSpeed);
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
