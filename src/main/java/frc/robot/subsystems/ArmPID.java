// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.team5024.lib.statemachines.StateMachine;
import com.team5024.lib.statemachines.StateMetadata;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmPID extends PIDSubsystem {
  /** Creates a new ArmPID. */

  private static ArmPID mInstance = null;

  public static ArmPID getInstance() {
    if (mInstance == null) {
      mInstance = new ArmPID();
    }

    return mInstance;
  }

  public enum State {
    AMP_POS,
    INTAKE,

  }

  protected StateMachine<State> stateMachine;

  private ArmPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    stateMachine = new StateMachine<>("Arm");
    stateMachine.setDefaultState(State.INTAKE, this::handleIntakeState);
    stateMachine.addState(State.AMP_POS, this::handleAmpState);

  }

  private void handleIntakeState(StateMetadata<State> metadata) {

  }

  private void handleAmpState(StateMetadata<State> metadata) {

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

}
