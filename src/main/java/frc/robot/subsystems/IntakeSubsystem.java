// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;   
import edu.wpi.first.wpilibj2.command.Commands; 

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX  intakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID, "Upper");
  private final TalonFX intakeFollowerMotor = new TalonFX(Constants.IntakeConstants.IntakeFollowerReversedMotorID, "Upper");

    TalonFXConfigurator leaderConfigurator = intakeMotor.getConfigurator();
    TalonFXConfigurator followerConfigurator = intakeFollowerMotor.getConfigurator();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    private static IntakeSubsystem instance;

  public IntakeSubsystem() {
    intakeFollowerMotor.setControl(new Follower(Constants.IntakeConstants.IntakeMotorID, MotorAlignmentValue.Opposed));

    // enable suply current limit
    limitConfigs.SupplyCurrentLimit = 60;
    limitConfigs.SupplyCurrentLimitEnable = true;

    leaderConfigurator.apply(limitConfigs);
    followerConfigurator.apply(limitConfigs);
  }

  public static IntakeSubsystem getInstance(){
    if (instance == null){
      instance = new IntakeSubsystem();
    }
    return instance;
  }
  
  public void set(double speed){
    intakeMotor.set(speed);
  }

  

  public void setHighSupplyLimit(int limit){
    limitConfigs.SupplyCurrentLimit = limit;
    
    leaderConfigurator.apply(limitConfigs);
    followerConfigurator.apply(limitConfigs);
  }

  public void stop(){
    intakeMotor.set(0);
  }
  @Override
  public void periodic() {
    double IntakeRPS = Math.abs(intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("IntakeSpeed", IntakeRPS);
  }
} 