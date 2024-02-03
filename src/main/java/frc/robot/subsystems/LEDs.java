package frc.robot.subsystems;

import com.team5024.lib.led.LEDController;
import com.team5024.lib.led.LEDPreset;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private static LEDs mInstance = null;
  private XboxController ledXboxController = new XboxController(0); // xbox controller is on port #0 on DriverStation
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
    // if "A" button on controller pressed
    if (ledXboxController.getAButtonPressed() == true) {
      // toggle logic for toggling A button on and off
      if (toggleLogic == true) {
        toggleLogic = false;
      } else {
        toggleLogic = true;
      }
    }

    if (toggleLogic == true) {
      // if "A" button pressed
      ledController.set(LEDPreset.Color1.kShot); //turn colour yellow
      System.out.println("LEDon");
    } else {
      // if "A" button is pressed again
      ledController.set(LEDPreset.Color2.kShot); //turn colour blue
      System.out.println("LEDoff");
    }
    // if limit switch is pressed, turn on fire (red)
    if (limitSwitch.get() == false) {
      ledController.set(LEDPreset.Fire.kParty);
    }
  }
}
