package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  private static Motor mInstance = null;

  private TalonFX mainMotor;

  public static Motor getInstance() {
    if (mInstance == null) {
      mInstance = new Motor();
    }

    return mInstance;
  }

  public enum State {
    Idle,
    Testing
  }

  PIDController pid = new PIDController(1.0, 0, 0);
  protected StateMachine<State> stateMachine;

  private Motor() {
    stateMachine = new StateMachine<>("Motor");
    stateMachine.setDefaultState(State.Idle, this::handleIdleState);
    stateMachine.addState(State.Testing, this::handleTestingState);
    mainMotor = new TalonFX(3);
  }

  private void handleIdleState(StateMetadata<State> metadata) {

  }

  private void handleTestingState(StateMetadata<State> metadata) {
    mainMotor.set(pid.calculate(mainMotor.getPosition().getValue(), 1024.0));
  }

  public void startTesting() {
    stateMachine.setState(State.Testing);
  }

  @Override
  public void periodic() {
    stateMachine.update();
  }
}
