// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
public class HopperSubsystem extends SubsystemBase {

  private final TalonFX hopperMotor = new TalonFX(Constants.HopperConstants.HopperMotorID, "Upper");

  TalonFXConfigurator HopperConfigurator = hopperMotor.getConfigurator();
  CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
  /** Creates a new Intake. */
  public HopperSubsystem() {
        // enable suply current limit
    limitConfigs.SupplyCurrentLimit = 20;
    limitConfigs.SupplyCurrentLimitEnable = true;

    HopperConfigurator.apply(limitConfigs);
  }

  public void set(double speed){
    hopperMotor.set(speed);
  }

  public void setHopperLimit(int limit){
    limitConfigs.SupplyCurrentLimit = limit;
    HopperConfigurator.apply(limitConfigs);
  }

  public void stop(){
    hopperMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}