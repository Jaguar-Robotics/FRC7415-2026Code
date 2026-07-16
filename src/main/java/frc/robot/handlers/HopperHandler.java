// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.ShooterHandler.ShooterState;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.HopperSubsystem;

public class HopperHandler extends SubsystemBase implements StateSubsystem {

    public enum HopperState implements State {
      FAST,
      SLOW,
      FASTOUT,
      SLOWOUT,
      OFF
  }

  private static HopperHandler instance;
  private final HopperSubsystem hopper = new HopperSubsystem();

  private HopperState desiredState = HopperState.OFF;
  private HopperState currentState = HopperState.OFF;
  /** Creates a new IntakeHandler. */
  private HopperHandler() {}

  public static HopperHandler getInstance(){
    if (instance == null){
      instance = new HopperHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof HopperState hopperState && desiredState != hopperState) {
        desiredState = hopperState;
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

      @Override
    public void update() {
        switch (desiredState) {
            case FAST:
                hopper.set(Constants.HopperConstants.FastRoll); 
                break;
            case SLOW:
                hopper.set(Constants.HopperConstants.SlowRoll);
                break;
            case FASTOUT:
                hopper.set(Constants.HopperConstants.FastOutRoll);
                break;
            case SLOWOUT:
                hopper.set(Constants.HopperConstants.SlowOutRoll);
                break;
            case OFF:
                hopper.stop();
                break;
            default:
                hopper.stop();
                break;
        }
        currentState = desiredState;
    }

      public HopperState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
