// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;   
import edu.wpi.first.wpilibj2.command.Commands; 
public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.IntakeMotorID);
  private final TalonFX intakeFollowerMotor = new TalonFX(Constants.IntakeConstants.IntakeFollowerReversedMotorID);
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeFollowerMotor.setControl(new Follower(Constants.IntakeConstants.IntakeMotorID, MotorAlignmentValue.Opposed));
  }

  public void set(double speed){
    //System.out.print(speed);
    intakeMotor.set(speed);
    //return Commands.run(() -> intakeMotor.set(speed));
  }

  public void stop(){
    intakeMotor.set(0);
    //return Commands.run(() -> intakeMotor.set(0.0));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
