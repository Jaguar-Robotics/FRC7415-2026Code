// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class KickerSubsystem extends SubsystemBase {

  private final TalonFX  kickerMotor = new TalonFX(Constants.IntakeConstants.KickerMotorID, "Upper");
  
  public KickerSubsystem() {
  }
  public void set(double speed){
    kickerMotor.set(speed);
    //System.out.println("peepeehouse");
  }

  public void stop(){
    kickerMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 