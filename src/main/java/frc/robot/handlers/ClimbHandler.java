// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.time.InstantSource;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.ShooterHandler.ShooterState;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;

public class ClimbHandler extends SubsystemBase implements StateSubsystem {

    public enum ClimbState implements State {
      OFF,
      LOW,
      MID,
      HIGH,
      LOWPULL
  }

  private static ClimbHandler instance;
  private final ClimbSubsystem climb = new ClimbSubsystem();

  private ClimbState desiredState = ClimbState.OFF;
  private ClimbState currentState = ClimbState.OFF;

  public Boolean extendedClimb = false;
  /** Creates a new IntakeHandler. */
  private ClimbHandler() {}

  public static ClimbHandler getInstance(){
    if (instance == null){
      instance = new ClimbHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof ClimbState climbState && desiredState != climbState) {
        desiredState = climbState;
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

      @Override
    public void update() {
        switch (desiredState) {
            case HIGH:
                climb.setHeight(Constants.ClimberConstants.HighSetPoint); 
                break;
            case LOW:
                climb.setHeight(Constants.ClimberConstants.LowSetPoint);
                if (climb.atTarget()) {
                  extendedClimb = true;
                }
                break;
            case LOWPULL:
                climb.setHeight(Constants.ClimberConstants.LowSetPointDown);
            case MID:
                climb.setHeight(Constants.ClimberConstants.MiddleSetPoint);
            case OFF:
                climb.stop();
                break;
            default:
                climb.stop();
                break;
        }
        currentState = desiredState;

    }

      public ClimbState getCurrentState() {
      return currentState;
  }

  
  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
