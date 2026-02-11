// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.DriveHandler.DriveState;
import frc.robot.handlers.ShooterHandler.ShooterState;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerHighHandler extends SubsystemBase implements StateSubsystem {

    public enum IndexerHighState implements State {
      FAST,
      SLOWINTAKE,
      FASTREVERSE,
      SLOWREVERSE,
      OFF
  }

  private static IndexerHighHandler instance;
  private final IndexerHighSubsystem index = new IndexerHighSubsystem();

  private IndexerHighState desiredState = IndexerHighState.OFF;
  private IndexerHighState currentState = IndexerHighState.OFF;
  /** Creates a new IntakeHandler. */
  private IndexerHighHandler() {}

  public static IndexerHighHandler getInstance(){
    if (instance == null){
      instance = new IndexerHighHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof IndexerHighState indexerState && desiredState != indexerState) {
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
            case FAST:
                index.set(Constants.IndexerConstants.FastRoll); 
                break;
            case SLOWINTAKE:
                index.set(Constants.IndexerConstants.SlowRoll);
                break;
            case FASTREVERSE:
                index.set(Constants.IndexerConstants.FastOutRoll);
                break;
            case SLOWREVERSE:
                index.set(Constants.IndexerConstants.SlowOutRoll);
                break;
            case OFF:
                index.stop(); 
                break;
            default:
                index.stop();
                break;
        }
        currentState = desiredState;
    }
  }

      public IndexerHighState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }
}
