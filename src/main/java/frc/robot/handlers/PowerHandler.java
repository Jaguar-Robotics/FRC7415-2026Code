// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    // What Superstructure most recently asked for (kept up to date even during an override)
    private PowerState requestedState = PowerState.IDLEINTAKE;
    // Active override, or null if none
    private PowerState powerOverride = null;

    @Override
    public void setDesiredState(State state) {
        if (!(state instanceof PowerState powerState)) return;

        requestedState = powerState;
        if (powerOverride != null) return; // override wins; we'll restore requestedState on release

        applyState(powerState);
    }

    /** Applies a power state, including the BEASTMODE end-of-match swap. */
    private void applyState(PowerState requested) {
        PowerState target = requested;
        if ((target == PowerState.STILLSCORE || target == PowerState.SOTM)
                && HubShiftUtil.getMatchTime() <= Constants.PowerManagerConstants.BeastModeTimeLimit
                && DriverStation.isTeleopEnabled()) {
            target = PowerState.BEASTMODE;
        }
        if (desiredState != target) {
            desiredState = target;
            handleStateTransition();
        }
    }

    /** Forces a power state (e.g. TURBODRIVE, INTAKEMAXXING) until cleared. */
    public void setPowerOverride(PowerState override) {
        powerOverride = override;
        applyState(override);
    }

    /** Clears the override only if it's the one that was set, then returns to whatever Superstructure wants. */
    public void clearPowerOverride(PowerState override) {
        if (powerOverride != override) return; // a different override has taken over; leave it alone
        powerOverride = null;
        applyState(requestedState);
    }

    /** Command factory: override while held, restore on release (or interrupt/disable). */
    public Command overrideWhileHeld(PowerState override) {
        return Commands.startEnd(
            () -> setPowerOverride(override),
            () -> clearPowerOverride(override));
    }

  // single thread = applies run in order, and config objects are only touched from that thread
  private final ExecutorService configExecutor = Executors.newSingleThreadExecutor(r -> {
      Thread t = new Thread(r, "PowerHandler-Config");
      t.setDaemon(true);
      return t;
  });

  private void setAllStates(int[] limitArray) {
      final int[] limits = limitArray.clone(); // snapshot so later changes can't affect a queued apply
      configExecutor.submit(() -> {
          try {
              drivetrain.setDTCurrentLimits(limits[0]);
              shooter.setShooterCurrentLimits(limits[1]);
              highIndexer.setHighIndexerLimit(limits[2]);
              lowIndexer.setLowIndexerLimit(limits[3]);
              hopper.setHopperLimit(limits[4]);
              intake.setHighSupplyLimit(limits[5]);
              kicker.setKickerSupplyCurrent(limits[6]);
              lintake.setMotorCurrentLimit(limits[7]);
          } catch (Exception e) {
              DogLog.log("PowerManager/ Config Error", e.toString());
          }
      });
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
    DogLog.log("PowerManager/ Current State", currentState.toString());
  }
} 
