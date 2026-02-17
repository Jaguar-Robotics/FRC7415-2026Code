// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BangBangShooterSubsystem extends SubsystemBase {
  /** Creates a new BangBangShooterSubsystem. */
  private final TalonFX ShooterMotor = new TalonFX(Constants.ShooterConstants.ShooterLeaderID, "Upper");
  private final TalonFX ShooterMotor2 = new TalonFX(Constants.ShooterConstants.ShooterFollowerID, "Upper");
  private final TalonFX ShooterMotorRev3 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversed2ID, "Upper");
  private final TalonFX ShooterMotorRev4 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversedID, "Upper");
    
  private final BangBangController controllerBangBang = new BangBangController();

  private final SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double targetVeloRPS = 0;
  private boolean shooterEnabled = false;
  
  public BangBangShooterSubsystem() {    
    
    ShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    ShooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    ShooterMotorRev3.setNeutralMode(NeutralModeValue.Coast);
    ShooterMotorRev4.setNeutralMode(NeutralModeValue.Coast);

    controllerBangBang.setTolerance(Constants.ShooterConstants.RPSTolarance);
  }


  private static BangBangShooterSubsystem instance;
  public static BangBangShooterSubsystem getInstance(){
      if (instance == null){
          instance = new BangBangShooterSubsystem();
      }
      return instance;
  }



  public void setTargetVelocity(double VelocityRPS) {
    if (VelocityRPS > Constants.ShooterConstants.RPSHardStop) {targetVeloRPS = Constants.ShooterConstants.RPSHardStop;}
    else {targetVeloRPS = VelocityRPS;}
    shooterEnabled = true;
  }

  public void coast(){
    shooterEnabled = false;
    targetVeloRPS = 0;
  }

  public boolean atTargetVelo(){
    return shooterEnabled && controllerBangBang.atSetpoint();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!shooterEnabled){
      ShooterMotor.setControl(voltageRequest.withOutput(0));
      ShooterMotor2.setControl(voltageRequest.withOutput(0));
      ShooterMotorRev3.setControl(voltageRequest.withOutput(0));
      ShooterMotorRev4.setControl(voltageRequest.withOutput(0));
      return;
    }

    double currentVelocity1RPS = ShooterMotor.getVelocity().getValueAsDouble();
    double currentVelocity2RPS = ShooterMotor2.getVelocity().getValueAsDouble();
    double currentVelocity3RPS = ShooterMotorRev3.getVelocity().getValueAsDouble();
    double currentVelocity4RPS = ShooterMotorRev4.getVelocity().getValueAsDouble();

    double bangBangVolts1 = controllerBangBang.calculate(currentVelocity1RPS, targetVeloRPS);
    double bangBangVolts2 = controllerBangBang.calculate(currentVelocity2RPS, targetVeloRPS);
    double bangBangVolts3 = controllerBangBang.calculate(currentVelocity3RPS, targetVeloRPS);
    double bangBangVolts4 = controllerBangBang.calculate(currentVelocity4RPS, targetVeloRPS);

    double  feedfowardVolts = 0.9 * feedFoward.calculate(targetVeloRPS);

    ShooterMotor.setControl(voltageRequest.withOutput(bangBangVolts1 + feedfowardVolts));
    ShooterMotor2.setControl(voltageRequest.withOutput(bangBangVolts2 + feedfowardVolts));
    ShooterMotorRev3.setControl(voltageRequest.withOutput(-(bangBangVolts3 + feedfowardVolts)));
    ShooterMotorRev4.setControl(voltageRequest.withOutput(-(bangBangVolts4 + feedfowardVolts)));

    SmartDashboard.putNumber("Shooter1Volts", bangBangVolts1 + feedfowardVolts);
    SmartDashboard.putNumber("Shooter2Volts", bangBangVolts2 + feedfowardVolts);
    SmartDashboard.putNumber("Shooter3Volts", -(bangBangVolts3 + feedfowardVolts));
    SmartDashboard.putNumber("Shooter4Volts", -(bangBangVolts4 + feedfowardVolts));

    SmartDashboard.putNumber("Shooter1RPS", currentVelocity1RPS);
    SmartDashboard.putNumber("Shooter2RPS", currentVelocity2RPS);
    SmartDashboard.putNumber("Shooter3RPS", currentVelocity3RPS);
    SmartDashboard.putNumber("Shooter4RPS", currentVelocity4RPS);

    SmartDashboard.putNumber("TargetRPS", targetVeloRPS);
  }
}
