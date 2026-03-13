package frc.robot.handlers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.DriveHandler.DriveState;
import frc.robot.handlers.ShooterHandler.ShooterState;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.HubShiftUtil;
import frc.robot.utils.HubShiftUtil.ShiftInfo;

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
      if (currentState != desiredState) {
          currentState = desiredState;
      }

      switch (currentState) {
          case FAST:
              if (HubShiftUtil.getShiftedShiftInfo().active()) {
                  index.set(Constants.IndexerConstants.FastRoll);
              } else {
                  index.stop();
              }
              break;
          case SLOWINTAKE:
              if (HubShiftUtil.getShiftedShiftInfo().active()) {
                  index.set(Constants.IndexerConstants.SlowRoll);
              } else {
                  index.stop();
              }
              break;
          case FASTREVERSE:
              index.set(Constants.IndexerConstants.FastOutRoll);
              break;
          case SLOWREVERSE:
              index.set(Constants.IndexerConstants.SlowOutRoll);
              break;
          case OFF:
          default:
              index.stop();
              break;
      }
        currentState = desiredState;
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