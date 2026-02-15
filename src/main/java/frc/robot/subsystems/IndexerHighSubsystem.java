// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;   
import edu.wpi.first.wpilibj2.command.Commands; 
public class IndexerHighSubsystem extends SubsystemBase {

  private final TalonFX indexerHighMotor = new TalonFX(Constants.IndexerConstants.HighIndexerMotorID);
  /** Creates a new Intake. */
  public IndexerHighSubsystem() {}

  public void set(double speed){
    //System.out.print(speed);
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
