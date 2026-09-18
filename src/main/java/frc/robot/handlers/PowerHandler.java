// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.utils.HubShiftUtil;

public class PowerHandler extends SubsystemBase implements StateSubsystem {

    
    public enum PowerState implements State {
      IDLEINTAKE, //norm
      STILLSCORE, //stationary shot
      SOTM, //sotm
      TURBODRIVE, //fast drive mode
      BEASTMODE, //prioritize feed for last x sec of match
      INTAKEMAXXING, //prioritize intake and kicker
      OUTAKE
  }

  private PowerState desiredState = PowerState.IDLEINTAKE;
  private PowerState currentState = PowerState.IDLEINTAKE;
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
        if ((desiredState == PowerState.STILLSCORE || desiredState == PowerState.SOTM) && HubShiftUtil.getMatchTime() <= Constants.PowerManagerConstants.BeastModeTimeLimit && DriverStation.isTeleopEnabled()){
          desiredState = PowerState.BEASTMODE;}
        handleStateTransition();
    }
  }

  private void setAllStates(int[] limitArray){
    drivetrain.setDTCurrentLimits(limitArray[0]);
    shooter.setShooterCurrentLimits(limitArray[1]);
    highIndexer.setHighIndexerLimit(limitArray[2]);
    lowIndexer.setLowIndexerLimit(limitArray[3]);
    hopper.setHopperLimit(limitArray[4]);
    intake.setHighSupplyLimit(limitArray[5]);
    kicker.setKickerSupplyCurrent(limitArray[6]);
    lintake.setMotorCurrentLimit(limitArray[7]);
  }

  @Override
  public void handleStateTransition() {
    update();
  }

    @Override
    public void update() {
        switch (desiredState) {
            case IDLEINTAKE:
              setAllStates(Constants.PowerManagerConstants.IdleIntake);
            break;
            case STILLSCORE:
              setAllStates(Constants.PowerManagerConstants.StillScore);
            break;
            case INTAKEMAXXING:
              setAllStates(Constants.PowerManagerConstants.IntakeMode);
            break;
            case SOTM:
              setAllStates(Constants.PowerManagerConstants.SOTMScore);
            break;
            case TURBODRIVE:
              setAllStates(Constants.PowerManagerConstants.TurboDrive);
            break;
            case BEASTMODE:
              setAllStates(Constants.PowerManagerConstants.BeastMode);
            break;
            case OUTAKE:
              setAllStates(Constants.PowerManagerConstants.Outtake);
            break;

        }
        currentState = desiredState;
    }

      public PowerState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    update();
    DogLog.log("PowerManager/ Current State", currentState.toString());
  }
} 
