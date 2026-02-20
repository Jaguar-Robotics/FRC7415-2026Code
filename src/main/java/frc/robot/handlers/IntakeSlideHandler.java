// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;

public class IntakeSlideHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakeSlideState implements State {
      OUT,
      MIDDLE,
      IN,
      SLOWIN,
      BRAKE,
      REZEROIN,
      REZEROOUT
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

  boolean isAtLowSetpoint = false;
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
                new SequentialCommandGroup(
                    intakeSlide.goToSetpoint(() -> Elevator.Setpoint.IN)
                        .until(() -> isAtLowSetpoint),
                    intakeSlide.calibrateZeroIn()
                ).schedule();
                break;
            case SLOWIN:
                new SequentialCommandGroup(
                  Commands.waitSeconds(3),
                  intakeSlide.manualDrive(() -> -0.125).until(intakeSlide.isHardStop)
                  ).schedule();
                //intakeSlide.goToSetpoint(() -> Elevator.Setpoint.Middle).schedule();
                break; 
            case BRAKE:
                intakeSlide.holdPosition();
                break;
            case REZEROIN:
                intakeSlide.calibrateZeroIn().schedule();
                break;
            case REZEROOUT:
                intakeSlide.calibrateZeroOut().schedule();
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
    isAtLowSetpoint = intakeSlide.isAtSetpoint(Elevator.Setpoint.IN);
    SmartDashboard.putBoolean("isAtLowSetpoint", isAtLowSetpoint);
    SmartDashboard.putString("IntakeSlide State", currentState.toString());
    // This method will be called once per scheduler run
  }
}