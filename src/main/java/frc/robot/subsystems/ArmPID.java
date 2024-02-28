package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team5024.lib.dashboard.SmarterDashboard;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmPID extends PIDSubsystem {
  private static ArmPID mInstance = null;

  private double destination = 0;

  private TalonFX armMotor;

  private DigitalInput armHallEffect;

  public static ArmPID getInstance() {
    if (mInstance == null) {
      mInstance = new ArmPID();
    }

    return mInstance;
  }

  public enum State {
    Zeroing,
    Moving,
  }

  protected StateMachine<State> stateMachine;

  private ArmPID() {

    super(new PIDController(ArmConstants.kP, 0, ArmConstants.kD));

    stateMachine = new StateMachine<>("Arm");
    stateMachine.setDefaultState(State.Zeroing, this::handleZeroing);
    stateMachine.addState(State.Moving, this::handleMoving);

    armMotor = new TalonFX(ArmConstants.armtalonID);

    armMotor.setPosition(0.0);

    armHallEffect = new DigitalInput(ArmConstants.armHallEffectID);
  }

  private void handleZeroing(StateMetadata<State> metadata) {
    setSetpoint(0);
    // disable();

    // if (armHallEffect.get() == false){

    // armMotor.set(.3);

    // }
    // else {

    // armMotor.setPosition(0.0);
    // stateMachine.setState(State.Moving);

    // }
    stateMachine.setState(State.Moving);
  }

  private void handleMoving(StateMetadata<State> metadata) {

    enable();
    // getController().setP(pEntry.getDouble(0));
    // getController().setD(dEntry.getDouble(0));

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

  public void setDestination(double desiredDestination) {

    destination = desiredDestination;
    stateMachine.setState(State.Moving);

  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    var speedCap = 5;
    armMotor.setVoltage(-MathUtil.clamp(output, -speedCap, speedCap));

    SmarterDashboard.putNumber("Arm/PID Output", output);
  }

  @Override
  public double getMeasurement() {
    return armMotor.getPosition().getValue() * ArmConstants.kEncoderDistancePerPulseRAD * -1;
  }

  @Override
  public void periodic() {
    super.periodic();

    stateMachine.update();

    SmarterDashboard.putNumber("Arm/Measurement (Degrees)", Units.radiansToDegrees(getMeasurement()));
    SmarterDashboard.putNumber("Arm/Measurement", getMeasurement());
    SmarterDashboard.putNumber("Arm/Motor Position", armMotor.getPosition().getValue());
    SmarterDashboard.putNumber("Arm/Motor Rotor Position", armMotor.getRotorPosition().getValue());

    double kP = SmarterDashboard.getNumber("Arm/kP", ArmConstants.kP);
    double kD = SmarterDashboard.getNumber("Arm/kD", ArmConstants.kD);

    if (SmarterDashboard.getBoolean("Arm/Debug", false)) {
      double debugSetpoint = SmarterDashboard.getNumber("Arm/Debug/Setpoint", getSetpoint());

      getController().setP(kP);
      getController().setD(kD);

      setSetpoint(debugSetpoint);
    }

    SmarterDashboard.putString("StateMachine/" + stateMachine.getName(), stateMachine.getCurrentState().toString());

    // Log subsystem to AK
    Logger.recordOutput("Subsystems/Arm/Current State", getCurrentState());
    Logger.recordOutput("Subsystems/Arm/Measurement", getMeasurement());
  }
}
