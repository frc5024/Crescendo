package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmPID extends PIDSubsystem {
  /** Creates a new ArmPID. */

  private static ArmPID mInstance = null;

  private double destination = 0;

  private TalonFX armMotor;

  private DigitalInput armHallEffect;

  private double speedCap;

  public static ArmPID getInstance() {
    if (mInstance == null) {
      mInstance = new ArmPID();
    }

    return mInstance;
  }

  public enum State {
    Moving
  }

  protected StateMachine<State> stateMachine;

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");

  private ArmPID() {

    super(new PIDController(ArmConstants.kP, 0, ArmConstants.kD));

    // destination = SETsetPoint.getDouble(0);
    // enables the states
    stateMachine = new StateMachine<>("Arm");
    stateMachine.addState(State.Moving, this::handleMoving);

    armMotor = new TalonFX(ArmConstants.armtalonID); // sets the TalonFX to the variable armMotor

    armMotor.setPosition(0.0); // sets the arm position to zero when starting the robot code

    armHallEffect = new DigitalInput(ArmConstants.armHallEffectID); // creates the hall effect sensor

    getController().setTolerance(Units.degreesToRadians(5)); // sets the tolerance to 5 degrees above and below the set
                                                             // point

    tab.addDouble("Current Setpoint", () -> getSetpoint());
    tab.addDouble("Deg Encoder", () -> Units.radiansToDegrees(getMeasurement()));
    tab.addDouble("GetMesurement", () -> getMeasurement());
    tab.addDouble("Rad encoder", () -> armMotor.getPosition().getValue());
    tab.addDouble("Raw encoder", () -> armMotor.getRotorPosition().getValue());

  }

  private void handleMoving(StateMetadata<State> metadata) {

    enable();

    setSetpoint(destination);

    double positionInUnits = Units.radiansToDegrees(getMeasurement());

    // if (positionInUnits >= (ArmConstants.midPoint) && armHallEffect.get() &&
    // destination <= positionInUnits
    // && positionInUnits <= ArmConstants.UpperLimit)
    // {
    // System.out.println("UPPPPPPPPPPPPPPPPP");

    // setSetpoint(destination);

    // } else if (positionInUnits <= (ArmConstants.midPoint) && armHallEffect.get()
    // &&
    // destination >= positionInUnits
    // && positionInUnits >=ArmConstants.intakeLimit)
    // {

    // System.out.println("DOWNNNNN");
    // setSetpoint(destination);

    // } else if (armHallEffect.get()) {

    // System.out.println("STOPPPPPPPPPPPPPPPPPPP");
    // setSetpoint(positionInUnits);
    // } else {
    // setSetpoint(destination);
    // }

  }

  public void Moving() {
    stateMachine.setState(State.Moving);

  }

  public void setStuff(double desiredDestination, double armSpeed) {

    speedCap = armSpeed;
    destination = desiredDestination;
    stateMachine.setState(State.Moving);

  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }

  @Override
  public void useOutput(double output, double setpoint) {

    System.out.println(output);
    armMotor.setVoltage(-MathUtil.clamp(output, -speedCap, speedCap));

  }

  @Override
  public double getMeasurement() {

    return armMotor.getPosition().getValue() * ArmConstants.kEncoderDistancePerPulseRAD * -1;
  }

  @Override
  public void periodic() {

    super.periodic();

    stateMachine.update();

  }
}
