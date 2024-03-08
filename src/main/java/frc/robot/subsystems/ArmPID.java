// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmPID extends PIDSubsystem {
  /** Creates a new ArmPID. */

  private static ArmPID mInstance = null;

  private double destination = 0;

  private TalonFX armMotor;

  private DigitalInput armHallEffect;

  private Timer climbTimer;

  public static ArmPID getInstance() {
    if (mInstance == null) {
      mInstance = new ArmPID();
    }

    return mInstance;
  }

  public enum State {
    Zeroing,
    Moving,
    Climbing,
    Stopped,
  }

  protected StateMachine<State> stateMachine;

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  GenericEntry pEntry = tab.add("SET P", ArmConstants.kP).getEntry();
  GenericEntry dEntry = tab.add("SET D", ArmConstants.kD).getEntry();
  GenericEntry maxSpeedEntry = tab.add("SET Max Speed", (5)).getEntry();
  GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();

  private ArmPID() {

    super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));

    stateMachine = new StateMachine<>("Arm");
    stateMachine.setDefaultState(State.Zeroing, this::handleZeroing);
    stateMachine.addState(State.Moving, this::handleMoving);
    stateMachine.addState(State.Climbing, this::handleClimbing);
    stateMachine.addState(State.Stopped, this::handleStopped);

    armMotor = new TalonFX(ArmConstants.armtalonID);

    armMotor.setPosition(0.0);

    armHallEffect = new DigitalInput(ArmConstants.armHallEffectID);

    climbTimer = new Timer();

    tab.addDouble("Current Setpoint", () -> getSetpoint());
    tab.addDouble("Deg Encoder", () -> Units.radiansToDegrees(getMeasurement()));
    tab.addDouble("GetMesurement", () -> getMeasurement());
    tab.addDouble("Raw getPos", () -> armMotor.getPosition().getValue());
    tab.addDouble("Raw getRotor", () -> armMotor.getRotorPosition().getValue());
    tab.addBoolean("Hall Effect", () -> armHallEffect.get());

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

  private void handleClimbing(StateMetadata<State> metadata) {
    if (metadata.isFirstRun()) {
      climbTimer.reset();
    }

    disable();
    armMotor.set(0.65);
    if (getMeasurement() <= (Constants.ArmConstants.intakeAngle + Units.degreesToRadians(2))) {
  

      // Ignore Timer Code if already stopped
      if (climbTimer.get() == 0) {
        climbTimer.start();
      }
    }
    if (climbTimer.get() >= 2) {
      stateMachine.setState(State.Stopped);
    }
  }

  private void handleStopped(StateMetadata<State> metadata) {
    disable();
    armMotor.stopMotor();
  }

  public void climb() {
    stateMachine.setState(State.Climbing);
  }

  public void Moving() {
    stateMachine.setState(State.Moving);

  }

  public void stop() {
    stateMachine.setState(State.Stopped);
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

    // System.out.println(output);
    var speedCap = 8; // maxSpeedEntry.getDouble(5);
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

    // getController().setP(pEntry.getDouble(ArmConstants.kP));
    // getController().setD(dEntry.getDouble(ArmConstants.kD));

    // Log subsystem to AK
    Logger.recordOutput("Subsystems/Arm/CurrentState", getCurrentState());
    Logger.recordOutput("Subsystems/Arm/DestinationDEG", Units.radiansToDegrees(destination));
    Logger.recordOutput("Subsystems/Arm/DestinationRAD", destination);
    Logger.recordOutput("Subsystems/Arm/MeasurementDEG", Units.radiansToDegrees(getMeasurement()));
    Logger.recordOutput("Subsystems/Arm/MeasurementRAD", getMeasurement());
    Logger.recordOutput("Subsystems/Arm/Velocity", armMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Arm/Voltage", armMotor.getMotorVoltage().getValueAsDouble());
  }
}
