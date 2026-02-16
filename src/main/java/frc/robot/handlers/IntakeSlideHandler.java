// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.handlers.DriveHandler.DriveState;
import frc.robot.handlers.IntakeSlideHandler.IntakeSlideState;
import frc.robot.handlers.ShooterHandler.ShooterState;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSlideHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakeSlideState implements State {
      OUT,
      MIDDLE,
      IN,
      SLOWIN,
      BRAKE
  }

  private static IntakeSlideHandler instance;
  private final Elevator intakeSlide = new Elevator(); //name it correct 


  private IntakeSlideState desiredState = IntakeSlideState.IN;
  private IntakeSlideState currentState = IntakeSlideState.IN;
  /** Creates a new IntakeHandler. */
  private IntakeSlideHandler() {}

  public static IntakeSlideHandler getInstance(){
    if (instance == null){
      instance = new IntakeSlideHandler();
    }
    return instance;
  }

  @Override
  public void setDesiredState(State state){
        if (state instanceof IntakeSlideState IntakeSlideState && desiredState != IntakeSlideState) {
        desiredState = IntakeSlideState;
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
            case OUT:
                intakeSlide.goToSetpoint(() -> Elevator.Setpoint.OUT).schedule();
                break;
            case MIDDLE:
                intakeSlide.goToSetpoint(() -> Elevator.Setpoint.Middle).schedule();
                break;
            case IN:
                 intakeSlide.goToSetpoint(() -> Elevator.Setpoint.IN).schedule();
                break;
            case SLOWIN:
                // break; CHANGE FOR REAL CODE LATER ITS NOT TOO HARD
            case BRAKE:
                intakeSlide.holdPosition();
                break;
            default:
                intakeSlide.holdPosition(); 
                break;
        }
        currentState = desiredState;
    }
  }

      public IntakeSlideState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    SmartDashboard.putString("IntakeSlide State", currentState.toString());
    // This method will be called once per scheduler run
  }
}