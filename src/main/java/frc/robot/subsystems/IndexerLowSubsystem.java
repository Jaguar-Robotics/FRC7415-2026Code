// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;   
import edu.wpi.first.wpilibj2.command.Commands; 
public class IndexerLowSubsystem extends SubsystemBase {

  private final TalonFX indexerLowMotor = new TalonFX(Constants.IndexerConstants.LowIndexerMotorID, "Upper");
  /** Creates a new Intake. */
  public IndexerLowSubsystem() {}

  public void set(double speed){
    indexerLowMotor.set(speed);
  }

  public void stop(){
    indexerLowMotor.set(0.0);
  }
  @Override
  public void periodic() {
    double lowindexRPS = Math.abs(indexerLowMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("indeherlowmotorRPS", lowindexRPS);
    // This method will be called once per scheduler run
  }
}
