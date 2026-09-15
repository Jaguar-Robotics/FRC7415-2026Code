// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
public class IndexerLowSubsystem extends SubsystemBase {


  private final TalonFX indexerLowMotor = new TalonFX(Constants.IndexerConstants.LowIndexerMotorID, "Upper");
  TalonFXConfigurator lowindexterConfigurator = indexerLowMotor.getConfigurator();
  CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
  
  /** Creates a new Intake. */
  public IndexerLowSubsystem() {
        // enable suply current limit
    limitConfigs.SupplyCurrentLimit = 20;
    limitConfigs.SupplyCurrentLimitEnable = true;

    lowindexterConfigurator.apply(limitConfigs);
  }

  public void set(double speed){
    indexerLowMotor.set(speed);
  }

  public void setLowIndexerLimit(int limit){
    limitConfigs.SupplyCurrentLimit = limit;
    lowindexterConfigurator.apply(limitConfigs);
  }

  public void stop(){
    indexerLowMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
