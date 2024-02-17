// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public static ArmPID getInstance() {
    if (mInstance == null) {
      mInstance = new ArmPID();
    }

    return mInstance;
  }

  public enum State {
    Moving,
  }

  protected StateMachine<State> stateMachine;

  ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  GenericEntry pEntry = tab.add("SET P", ArmConstants.kP).getEntry();
  GenericEntry dEntry = tab.add("SET D", ArmConstants.kD).getEntry();
  GenericEntry maxSpeedEntry = tab.add("SET Max Speed", 0.2).getEntry();
  GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();

  private ArmPID() {

    super(new PIDController(0, 0, 0));

    stateMachine = new StateMachine<>("Arm");
    stateMachine.setDefaultState(State.Moving, this::handleMoving);

    armMotor = new TalonFX(ArmConstants.armtalonID);
    armMotor.setPosition(0.0);

    armHallEffect = new DigitalInput(ArmConstants.armHallEffectID);

    tab.addDouble("Current Setpoint", () -> getSetpoint());
    tab.addDouble("Deg Encoder", () -> Units.radiansToDegrees(getMeasurement()));
    tab.addDouble("GetMesurement", () -> getMeasurement());
    tab.addDouble("Raw getPos", () -> armMotor.getPosition().getValue());
    tab.addDouble("Raw getRotor", () -> armMotor.getRotorPosition().getValue());

  }

  private void handleMoving(StateMetadata<State> metadata) {

    enable();

    destination = Units.degreesToRadians(SETsetPoint.getDouble(0.0));

    setSetpoint(destination);

    // double positionInUnits = getMeasurement();

    // if (positionInUnits >= (ArmConstants.midPoint) && armHallEffect.get() &&
    // destination <= positionInUnits && positionInUnits <= ArmConstants.UpperLimit)
    // {
    // System.out.println("UPPPPPPPPPPPPPPPPP");

    // setSetpoint(destination);

    // } else if (positionInUnits <= (ArmConstants.midPoint) &&
    // armHallEffect.get()&&
    // destination >= positionInUnits && positionInUnits >=
    // ArmConstants.intakeLimit) {

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

    // destination = desiredDestination;
    // enable();

  }

  public State getCurrentState() {
    return stateMachine.getCurrentState();
  }

  @Override
  public void useOutput(double output, double setpoint) {

    System.out.println(output);
    var speedCap = maxSpeedEntry.getDouble(0.1);
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

    getController().setP(pEntry.getDouble(ArmConstants.kP));
    getController().setD(dEntry.getDouble(ArmConstants.kD));
  }
}
