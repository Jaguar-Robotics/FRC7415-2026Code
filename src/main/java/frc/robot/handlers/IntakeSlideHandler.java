// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;

public class IntakeSlideHandler extends SubsystemBase implements StateSubsystem {

    public enum IntakeSlideState implements State {
      OUT,
      MIDDLE,
      IN,
      SLOWIN,
      BRAKE,
      REZEROIN,
      REZEROOUT,
      OSCILLATE
  }

  private static IntakeSlideHandler instance;
  private final Elevator intakeSlide = new Elevator(); //name it correct 

  
  Trigger intakeAboveFive = new Trigger(() -> intakeSlide.getPosition().in(Rotations) > 5.0);


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
  boolean isAtOutSetpoint = false;
      @Override
    public void update() {
      if((currentState != desiredState)){
        switch (desiredState) {
            case OUT:
            /*
                new SequentialCommandGroup(
                    intakeSlide.goToSetpoint(() -> Elevator.Setpoint.OUT)
                        .until(() -> isAtOutSetpoint),
                    intakeSlide.holdPosition()
                ).schedule();
              */
              intakeSlide.goToSetpoint(() -> Elevator.Setpoint.OUT).schedule();
                break;
            case MIDDLE:
                intakeSlide.goToSetpoint(() -> Elevator.Setpoint.Middle).schedule();
                break;
            case IN: //In also re-zeros it once it reaches "in"
                new SequentialCommandGroup(
                    intakeSlide.goToSetpoint(() -> Elevator.Setpoint.IN)
                        .until(() -> isAtLowSetpoint),
                    intakeSlide.calibrateZeroIn()
                ).schedule();
                break;
            case SLOWIN: 
                new SequentialCommandGroup(
                  Commands.waitSeconds(0.25),
                  intakeSlide.manualDrive(() -> -0.2).until(() -> intakeSlide.isHardStop.getAsBoolean() && isAtLowSetpoint).withTimeout( 5)
                  ).schedule();
                break; 
            case OSCILLATE: 
                new SequentialCommandGroup(
                  intakeSlide.goToSetpoint(() -> Elevator.Setpoint.OUT).withTimeout(0.5),
                  Commands.waitSeconds(0.5),
                  intakeSlide.goToSetpoint(() -> Elevator.Setpoint.IN).withTimeout(0.5)
                ).repeatedly().schedule();
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
    isAtOutSetpoint = intakeSlide.isAtSetpoint(Elevator.Setpoint.OUT);
    SmartDashboard.putBoolean("isAtLowSetpoint", isAtLowSetpoint);
    SmartDashboard.putString("IntakeSlide State", currentState.toString());
    // This method will be called once per scheduler run
  }
}