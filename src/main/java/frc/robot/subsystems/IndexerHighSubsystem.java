// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
public class IndexerHighSubsystem extends SubsystemBase {

  private final TalonFX indexerHighMotor = new TalonFX(Constants.IndexerConstants.HighIndexerMotorID, "Upper");
  private static IndexerHighSubsystem instance;

  TalonFXConfigurator HighindexterConfigurator = indexerHighMotor.getConfigurator();
  CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

  /** Creates a new Intake. */
  public IndexerHighSubsystem() {
    // enable suply current limit
    limitConfigs.SupplyCurrentLimit = 20;
    limitConfigs.SupplyCurrentLimitEnable = true;

    HighindexterConfigurator.apply(limitConfigs);
  }

  public static IndexerHighSubsystem getInstance(){
    if (instance == null){
      instance = new IndexerHighSubsystem();
    }
    return instance;
  }

  public void setHighIndexerLimit(int limit){
    limitConfigs.SupplyCurrentLimit = limit;
    HighindexterConfigurator.apply(limitConfigs);
  }

  public void set(double speed){
    indexerHighMotor.set(speed);
  }

  public void stop(){
    indexerHighMotor.set(0.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
