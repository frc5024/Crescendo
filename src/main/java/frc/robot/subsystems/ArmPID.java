// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ArmPID extends PIDSubsystem {
  /** Creates a new ArmPID. */

  private static ArmPID mInstance = null;

  public double destination = 0;

  private TalonSRX armMotor;
  private Encoder armEncoder;

  private DigitalInput armIntakeHallEffect;
  private DigitalInput armBackHallEffect;

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

  private ArmPID() {

    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    stateMachine = new StateMachine<>("Arm");
    stateMachine.setDefaultState(State.Moving, this::handleMoving);

    armMotor = new TalonSRX(Constants.ArmConstants.armtalonID);

    armEncoder = new Encoder(Constants.ArmConstants.encoderChannels[0], Constants.ArmConstants.encoderChannels[1],
        false, Encoder.EncodingType.k4X);
    armEncoder.setDistancePerPulse(Constants.ArmConstants.encoderDistancePerPulse);
    armEncoder.setMinRate(Constants.ArmConstants.encoderMinRate);

    armBackHallEffect = new DigitalInput(Constants.ArmConstants.armBackHallEffectID);
    armIntakeHallEffect = new DigitalInput(Constants.ArmConstants.armIntakeHallEffectID);

  }

  private void handleMoving(StateMetadata<State> metadata) {

    if (armBackHallEffect.get() == true) {

      setSetpoint(armEncoder.get());

    } else if (armIntakeHallEffect.get() == true) {

      armEncoder.reset();
      setSetpoint(armEncoder.get());
    } else {

      setSetpoint(destination);
    }

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
    // Use the output here
    armMotor.set(TalonSRXControlMode.PercentOutput, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here

    return armEncoder.getDistance();
  }

}
