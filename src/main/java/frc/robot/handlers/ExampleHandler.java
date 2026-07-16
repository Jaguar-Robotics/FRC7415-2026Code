// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleHandler extends SubsystemBase implements StateSubsystem {

    public enum ExampleState implements State {
      ON,
      OFF
  }

  private static ExampleHandler instance;
  private final ExampleSubsystem index = new ExampleSubsystem();

  private ExampleState desiredState = ExampleState.OFF;
  private ExampleState currentState = ExampleState.OFF;
  /** Creates a new IntakeHandler. */
  private ExampleHandler() {}

  public static ExampleHandler getInstance(){
    if (instance == null){
      instance = new ExampleHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof ExampleState indexerState && desiredState != indexerState) {
        desiredState = indexerState;
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

      @Override
    public void update() {
      if((currentState != desiredState)){
        switch (desiredState) {
            case ON:
                index.set(Constants.ExampleSubsystemConstants.fastSpeed); //this number could be hard coded, but when it's a variable it is alot easier to change (see Constants file)
                break;
            case OFF:
                index.set(0);
                break;
        }
        currentState = desiredState;
    }
  }

      public ExampleState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
