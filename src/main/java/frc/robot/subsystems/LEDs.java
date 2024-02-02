package frc.robot.subsystems;

import com.team5024.lib.led.LEDController;
import com.team5024.lib.led.LEDPreset;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private static LEDs mInstance = null;
  private XboxController ledXboxController = new XboxController(0); // xbox controller is on port #0 to control LEDs
  private DigitalInput limitSwitch = new DigitalInput(0); // limit switch is on DIO #0
  private LEDController ledController;
  public boolean toggleLogic;

  public static LEDs getInstance() {
    if (mInstance == null) {
      mInstance = new LEDs();
    }
    return mInstance;
  }

  private LEDs() {
    ledController = new LEDController(Constants.LEDConstants.ledPort);
  }

  @Override
  public void periodic() {
    // if a button on controller pressed
    if (ledXboxController.getAButtonPressed() == true) {
      // toggle logic for toggling A button on and off
      if (toggleLogic == true) {
        toggleLogic = false;
      } else {
        toggleLogic = true;
      }
    }
    // if B button is pressed, it resets back to normal colour
    if (ledXboxController.getBButton()) {
      if (limitSwitch.get() == true) {
        ledController.set(LEDPreset.Fire.kRainbow);
      } else {
        ledController.set(LEDPreset.Color1.kBreathFast);
      }
    }
    if (toggleLogic == true) {
      // if a button pressed
      ledController.set(LEDPreset.Color1.kBreathFast);
      System.out.println("LEDon");
    } else {
      // if a button is pressed again
      ledController.set(LEDPreset.Color2.kBreathSlow);
      System.out.println("LEDoff");
    }
    // if limit switch on, turn on fire (red)
    if (limitSwitch.get() == false) {
      ledController.set(LEDPreset.Fire.kRainbow);
    }
  }
}
