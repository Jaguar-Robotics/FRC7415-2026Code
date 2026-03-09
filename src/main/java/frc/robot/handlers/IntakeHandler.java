// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHandler extends SubsystemBase implements StateSubsystem {

    
    public enum IntakeState implements State {
      FASTINTAKE,
      SLOWINTAKE,
      FASTREVERSE,
      SLOWREVERSE,
      OFF
  }

  private static IntakeHandler instance;
  private Elevator lintake;
  private IntakeSubsystem intake = new IntakeSubsystem();
  private IntakeState desiredState = IntakeState.OFF;
  private IntakeState currentState = IntakeState.OFF;

    public void initialize(Elevator lintake) {
    this.lintake = lintake;
    }


  private IntakeHandler() {}

  public static IntakeHandler getInstance(){
      if (instance == null){
          instance = new IntakeHandler();
      }
      return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof IntakeState intakeState && desiredState != intakeState) {
        desiredState = intakeState;
        handleStateTransition();
    }
  }

  @Override
  public void handleStateTransition() {
    update();
  }

    @Override
    public void update() {
        switch (desiredState) {
            case FASTINTAKE:
                intake.set(Constants.IntakeConstants.FastIntake);
                break;
            case SLOWINTAKE:
                intake.set(Constants.IntakeConstants.SlowIntake);
                break;
            case FASTREVERSE:
                intake.set(Constants.IntakeConstants.FastReverse);
                break;
            case SLOWREVERSE:
                intake.set(Constants.IntakeConstants.SlowReverse);
                break;
            case OFF:
                intake.stop();
                break;
            default:
                intake.stop();
                break;
        }
        currentState = desiredState;
    }

      public IntakeState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("IntakeHandlerState", currentState.toString());
    update();
    // This method will be called once per scheduler run
  }
} 
