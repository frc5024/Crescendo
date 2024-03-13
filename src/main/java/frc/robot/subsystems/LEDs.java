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
    // private DigitalInput limitSwitch = new DigitalInput(0); // limit switch is on
    // DIO #0
    private LEDController ledController;
    private Shooter shooterInstance;
    Timer timer;
    int flashCount = 0;

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
        stateMachine = new StateMachine<>("LEDS");
        stateMachine.setDefaultState(State.Idle, this::handleIdleState);
        stateMachine.addState(State.Holding, this::handleHoldingState);
        stateMachine.addState(State.Warming, this::handleWarmingState);
        ledController = new LEDController(Constants.LEDConstants.ledPort);
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
            if (timer.hasElapsed(0.5)) {
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {
                ledController.set(LEDPreset.Solid.kGreen);
                System.out.println("PIECE IN");
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
        if (flashCount < 10) {
            if (timer.hasElapsed(0.5)) {
                flashCount++;
                timer.restart();
            }

            if (flashCount % 2 == 0) {
                ledController.set(LEDPreset.Solid.kOrange);
                System.out.println("PIECE IN");
            } else {
                ledController.set(LEDPreset.Solid.kBlack);
            }
        } else {
            ledController.set(LEDPreset.Solid.kOrange);
        }
    }

    @Override
    public void periodic() {

        if (shooterInstance.warmedUp() == true) {
            stateMachine.setState(State.Warming);

            if (shooterInstance.isLineBroken() == true) {
                stateMachine.setState(State.Holding);
            }
            if (shooterInstance.isLineBroken() == false) {
                stateMachine.setState(State.Idle);
            }

        }
    }
}