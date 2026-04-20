// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
public class IndexerHighSubsystem extends SubsystemBase {

  private final TalonFX indexerHighMotor = new TalonFX(Constants.IndexerConstants.HighIndexerMotorID, "Upper");
  /** Creates a new Intake. */
  public IndexerHighSubsystem() {}

  public void set(double speed){
    indexerHighMotor.set(speed);
  }

  public void stop(){
    indexerHighMotor.set(0.0);
  }
  @Override
  public void periodic() {
    double highindexRPS = Math.abs(indexerHighMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("indeherHighmotor", highindexRPS);
    // This method will be called once per scheduler run
  }
}
