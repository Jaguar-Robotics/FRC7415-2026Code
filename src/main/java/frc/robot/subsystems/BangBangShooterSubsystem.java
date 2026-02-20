// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private double targetVeloRPS1 = 0;
  private double targetVeloRPS2 = 0;
  private double targetVeloRPS3 = 0;
  private double targetVeloRPS4 = 0;

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
    if (VelocityRPS > Constants.ShooterConstants.RPSHardStop) {
      targetVeloRPS1 = Constants.ShooterConstants.RPSHardStop;
      targetVeloRPS2 = Constants.ShooterConstants.RPSHardStop;
      targetVeloRPS3 = Constants.ShooterConstants.RPSHardStop;
      targetVeloRPS4 = Constants.ShooterConstants.RPSHardStop;
    }

    else {
      targetVeloRPS1 = VelocityRPS;
      targetVeloRPS2 = VelocityRPS;
      targetVeloRPS3 = VelocityRPS;
      targetVeloRPS4 = VelocityRPS;
    }
    shooterEnabled = true;
  }

  public void setTargetVeloDistance(double  distMeters) {
    double distInches = distMeters * 39.3701;
    //https://www.desmos.com/calculator/lvyi23hstj
    targetVeloRPS1 = 0.00239451 * Math.pow(distInches, 2) - 0.0705712*distInches + 40.63813;
    targetVeloRPS2 = 0.000748127 * Math.pow(distInches, 2) + 0.142013*distInches + 35.372;
    targetVeloRPS3 = 0.0065788 * Math.pow(distInches, 2) - 0.705641*distInches + 61.31564;
    targetVeloRPS4 = 0.000480753 * Math.pow(distInches, 2) + 0.280501*distInches + 27.90482;

    if (targetVeloRPS1 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS1 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS2 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS2 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS3 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS3 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS4 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS4 = Constants.ShooterConstants.RPSHardStop;}

    shooterEnabled = true;
  }

  public void coast(){
    shooterEnabled = false;
    targetVeloRPS1 = 0;
    targetVeloRPS2 = 0;
    targetVeloRPS3 = 0;
    targetVeloRPS4 = 0;
  }

 public boolean atTargetVelo() {
  boolean atTargBelo = Math.abs(ShooterMotor.getVelocity().getValueAsDouble())  >=  targetVeloRPS1;
  return atTargBelo;
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

    double bangBangVolts1 = controllerBangBang.calculate(currentVelocity1RPS, targetVeloRPS1);
    double bangBangVolts2 = controllerBangBang.calculate(currentVelocity2RPS, targetVeloRPS2);
    double bangBangVolts3 = controllerBangBang.calculate(currentVelocity3RPS, targetVeloRPS3);
    double bangBangVolts4 = controllerBangBang.calculate(currentVelocity4RPS, targetVeloRPS4);

    double  feedfowardVolts1 = 0.9 * feedFoward.calculate(targetVeloRPS1);
    double  feedfowardVolts2 = 0.9 * feedFoward.calculate(targetVeloRPS2);
    double  feedfowardVolts3 = 0.9 * feedFoward.calculate(targetVeloRPS3);
    double  feedfowardVolts4 = 0.9 * feedFoward.calculate(targetVeloRPS4);

    ShooterMotor.setControl(voltageRequest.withOutput(-(bangBangVolts1 + feedfowardVolts1))); //idk bru
    ShooterMotor2.setControl(voltageRequest.withOutput(-(bangBangVolts2 + feedfowardVolts2))); //backwards in phy tuner
    ShooterMotorRev3.setControl(voltageRequest.withOutput(-(bangBangVolts3 + feedfowardVolts3))); //-
    ShooterMotorRev4.setControl(voltageRequest.withOutput(-(bangBangVolts4 + feedfowardVolts4))); //-

    SmartDashboard.putNumber("Shooter1Volts", bangBangVolts1 + feedfowardVolts1);
    SmartDashboard.putNumber("Shooter2Volts", bangBangVolts2 + feedfowardVolts2);
    SmartDashboard.putNumber("Shooter3Volts", bangBangVolts3 + feedfowardVolts3);
    SmartDashboard.putNumber("Shooter4Volts", bangBangVolts4 + feedfowardVolts4);

    SmartDashboard.putNumber("Shooter1RPS", Math.abs(currentVelocity1RPS));
    SmartDashboard.putNumber("Shooter2RPS", Math.abs(currentVelocity2RPS));
    SmartDashboard.putNumber("Shooter3RPS", Math.abs(currentVelocity3RPS));
    SmartDashboard.putNumber("Shooter4RPS", Math.abs(currentVelocity4RPS));

    SmartDashboard.putNumber("TargetRPS1", targetVeloRPS1);
    SmartDashboard.putNumber("TargetRPS2", targetVeloRPS2);
    SmartDashboard.putNumber("TargetRPS3", targetVeloRPS3);
    SmartDashboard.putNumber("TargetRPS4", targetVeloRPS4);
  }
}
