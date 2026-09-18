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
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class PowerHandler extends SubsystemBase implements StateSubsystem {

    
    public enum PowerState implements State {
      IDLE, //norm
      STILLSCORE, //stationary shot
      SOTM, //sotm
      TURBODRIVE, //fast drive mode
      BEASTMODE, //prioritize feed for last x sec of match
      AUTO //lowk im not using ts
  }

  private PowerState desiredState = PowerState.AUTO;
  private PowerState currentState = PowerState.AUTO;
  private static PowerHandler instance;
  
  CommandSwerveDrivetrain drivetrain;
  BangBangShooterSubsystem shooter;
  IndexerHighSubsystem highIndexer;
  IndexerLowSubsystem lowIndexer;
  HopperSubsystem hopper;
  IntakeSubsystem intake;
  KickerSubsystem kicker;
  Elevator lintake;
  //Drive/ Shooter/ HighIndex/ LowIndex/ HopperFloor/ Intake/ Kicker/ Lintake (all supply upper limits)
  public void initialize(  
          CommandSwerveDrivetrain drivetrain,
          BangBangShooterSubsystem shooter,
          IndexerHighSubsystem highIndexer,
          IndexerLowSubsystem lowIndexer,
          HopperSubsystem hopper,
          IntakeSubsystem intake,
          KickerSubsystem kicker,
          Elevator lintake) 
          {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.highIndexer = highIndexer;
    this.lowIndexer = lowIndexer;
    this.hopper = hopper;
    this.intake = intake;
    this.kicker = kicker;
    this.lintake = lintake;
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

  private void setAllStates(int[] limitArray){
    drivetrain.setDTCurrentLimits(limitArray[1]);
    shooter.setShooterCurrentLimits(limitArray[2]);
    highIndexer.setHighIndexerLimit(limitArray[3]);
    lowIndexer.setLowIndexerLimit(limitArray[4]);
    hopper.setHopperLimit(limitArray[5]);
    intake.setHighSupplyLimit(limitArray[6]);
    kicker.setKickerSupplyCurrent(limitArray[7]);
    lintake.setMotorCurrentLimit(limitArray[8]);
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
