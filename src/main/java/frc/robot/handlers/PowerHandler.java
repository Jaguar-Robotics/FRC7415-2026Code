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
import frc.robot.handlers.IntakeHandler.IntakeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class PowerHandler extends SubsystemBase implements StateSubsystem {

    
    public enum PowerState implements State {
      IDLE,
      STILLSCORE,
      SOTM,
      TURBODRIVE,
      BEASTMODE,
      AUTO
  }

  private PowerState desiredState = PowerState.AUTO;
  private PowerState currentState = PowerState.AUTO;
  private static PowerHandler instance;

    public void initialize() {
    }


  private PowerHandler() {}

  public static PowerHandler getInstance(){
      if (instance == null){
          instance = new PowerHandler();
      }
      return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof PowerState powerState && desiredState != powerState) {
        desiredState = powerState;
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
            case IDLE:
            break;
        }
        currentState = desiredState;
    }

      public PowerState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("IntakeHandlerState", currentState.toString());
    update();
    // This method will be called once per scheduler run
  }
} 
