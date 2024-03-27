package frc.robot.subsystems;

import com.team5024.lib.led.LEDController;
import com.team5024.lib.led.LEDPreset;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    private static LEDs mInstance = null;
    private LEDController ledController;

    private Shooter shooterInstance = Shooter.getInstance();

    private Timer timer = new Timer();
    private int flashCount = 0;

    public enum State {
        Idle,
        Holding,
        Warming,
    }

    protected StateMachine<State> stateMachine;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private LEDs() {
        ledController = new LEDController(Constants.LEDConstants.ledPort);

        stateMachine = new StateMachine<>("LEDs");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Holding, this::handleHoldingState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
    }

    private void handleIdleState(StateMetadata<State> metadata) {
        ledController.set(LEDPreset.Solid.kRed);
    }

    private void handleHoldingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            flashCount = 0;
            timer.reset();
            timer.restart();
        }
        if (flashCount < 10) {
            if (timer.hasElapsed(0.1)) {
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {
                ledController.set(LEDPreset.Solid.kGreen);
            } else {
                ledController.set(LEDPreset.Solid.kBlack);
            }
        } else {
            ledController.set(LEDPreset.Solid.kGreen);
        }
    }

    private void handleWarmingState(StateMetadata<State> metadata) {
        if (metadata.isFirstRun()) {
            flashCount = 0;
            timer.reset();
            timer.restart();
        }

        if (timer.hasElapsed(0.1)) {
            flashCount++;
            timer.restart();
        }

        if (flashCount % 2 == 0) {
            ledController.set(LEDPreset.Solid.kOrange);
        } else {
            ledController.set(LEDPreset.Solid.kBlack);
        }

    }

    @Override
    public void periodic() {
        if (shooterInstance.warmedUp()) {
            stateMachine.setState(State.Warming);
        } else if (shooterInstance.isLineBroken()) {
            stateMachine.setState(State.Holding);
        } else {
            stateMachine.setState(State.Idle);
        }

        stateMachine.update();
    }
}