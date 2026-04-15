// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class BangBangShooterSubsystem extends SubsystemBase {
  /** Creates a new BangBangShooterSubsystem. */
  private final TalonFX ShooterMotor = new TalonFX(Constants.ShooterConstants.ShooterLeaderID, "Upper");
  private final TalonFX ShooterMotor2 = new TalonFX(Constants.ShooterConstants.ShooterFollowerID, "Upper");
  private final TalonFX ShooterMotorRev3 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversed2ID, "Upper");
  private final TalonFX ShooterMotorRev4 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversedID, "Upper");

  private final BangBangController controllerBangBang = new BangBangController();

  private static final InterpolatingDoubleTreeMap Shooter1Map = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap Shooter2Map = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap Shooter3Map = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap Shooter4Map = new InterpolatingDoubleTreeMap();
  
  //inches to center hub from Robot orign , RPS
  static {
    Shooter1Map.put(138.4 ,105.0);
    Shooter1Map.put(121.4 ,77.0);
    Shooter1Map.put(104.6 ,70.0);
    /*
    Shooter1Map.put(74.3, 56.0);
    Shooter1Map.put(60.5, 52.0);
    Shooter1Map.put(52.1, 50.0);
    */
    
    Shooter1Map.put(85.8, 64.0);
    Shooter1Map.put(69.0, 60.0);
    Shooter1Map.put(56.7,52.0);
    


    Shooter2Map.put(138.4 ,105.0);
    Shooter2Map.put(121.4 ,77.0);
    Shooter2Map.put(104.6 ,70.0);
    Shooter2Map.put(85.8, 64.0);
    Shooter2Map.put(69.0, 60.0);
    Shooter2Map.put(56.7,52.0);


    Shooter3Map.put(138.4 ,105.0);
    Shooter3Map.put(121.4 ,77.0);
    Shooter3Map.put(104.6 ,70.0);
    Shooter3Map.put(85.8, 64.0);
    Shooter3Map.put(69.0, 60.0);
    Shooter3Map.put(56.7,52.0);


    Shooter4Map.put(138.4 ,105.0);
    Shooter4Map.put(121.4 ,77.0);
    Shooter4Map.put(104.6 ,70.0);
    Shooter4Map.put(85.8, 64.0);
    Shooter4Map.put(69.0, 60.0);
    Shooter4Map.put(56.7,52.0);

  }

 
  private final SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double targetVeloRPS1 = 0;
  private double targetVeloRPS2 = 0;
  private double targetVeloRPS3 = 0;
  private double targetVeloRPS4 = 0;

  private double ShooterMult = 0.95; //0.95



  private boolean shooterEnabled = false;

  private boolean MaxRPM = false;
  private boolean MultiplierOn = true;
  
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

  public void changeShooterMult(double multAdd){
    ShooterMult += multAdd;
  }

  public void toggleShooterMult(){
    if (MultiplierOn) {MultiplierOn = false;}
    if (!MultiplierOn) {MultiplierOn = true;}
  }

  public void resetShooterMult(){
    ShooterMult = 1.0; //change to 0.95
  }

  // without mult
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
  // with mult
  public void setTargetVeloDistance(double  distance) {//IN METERS
    double inches = distance * 39.3701;
    targetVeloRPS1 = Shooter1Map.get(inches) * ShooterMult; //get rid of shootermult
    targetVeloRPS2 = Shooter2Map.get(inches) * ShooterMult;
    targetVeloRPS3 = Shooter3Map.get(inches) * ShooterMult;
    targetVeloRPS4 = Shooter4Map.get(inches) * ShooterMult;

    if (targetVeloRPS1 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS1 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS2 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS2 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS3 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS3 = Constants.ShooterConstants.RPSHardStop;}
    if (targetVeloRPS4 >= Constants.ShooterConstants.RPSHardStop) { targetVeloRPS4 = Constants.ShooterConstants.RPSHardStop;}

    if (targetVeloRPS1 >= 104.0){
      MaxRPM = true;
    }
    else{
      MaxRPM = false;
    }

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
  if (RobotBase.isSimulation()) return true;
  boolean atTargBelo = Math.abs(ShooterMotor.getVelocity().getValueAsDouble())  >=  targetVeloRPS1-2.0;
  return atTargBelo;
  }

  public boolean atTargetVeloPassing() {
  boolean atTargBelo = (Math.abs(ShooterMotor.getVelocity().getValueAsDouble())  >=  targetVeloRPS1-2.0) || ShooterMotor.getVelocity().getValueAsDouble() >= 98.0;
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

    double currentVelocity1RPS = Math.abs(ShooterMotor.getVelocity().getValueAsDouble());
    double currentVelocity2RPS = Math.abs(ShooterMotor2.getVelocity().getValueAsDouble());
    double currentVelocity3RPS = Math.abs(ShooterMotorRev3.getVelocity().getValueAsDouble());
    double currentVelocity4RPS = Math.abs(ShooterMotorRev4.getVelocity().getValueAsDouble());

    double bangBangVolts1 = controllerBangBang.calculate(currentVelocity1RPS, targetVeloRPS1)*12;
    double bangBangVolts2 = controllerBangBang.calculate(currentVelocity2RPS, targetVeloRPS2)*12;
    double bangBangVolts3 = controllerBangBang.calculate(currentVelocity3RPS, targetVeloRPS3)*12;
    double bangBangVolts4 = controllerBangBang.calculate(currentVelocity4RPS, targetVeloRPS4)*12;

    double feedfowardVolts1 = feedFoward.calculate(targetVeloRPS1)*0.9; //*0.9 */
    double feedfowardVolts2 = feedFoward.calculate(targetVeloRPS2)*0.9;
    double feedfowardVolts3 = feedFoward.calculate(targetVeloRPS3)*0.9;
    double feedfowardVolts4 = feedFoward.calculate(targetVeloRPS4)*0.9;

    ShooterMotor.setControl(voltageRequest.withOutput(-(bangBangVolts1 + feedfowardVolts1)).withEnableFOC(false)); //idk bru bang bang volts 1
    ShooterMotor2.setControl(voltageRequest.withOutput(-(bangBangVolts2 + feedfowardVolts2)).withEnableFOC(false)); //backwards in phy tuner
    ShooterMotorRev3.setControl(voltageRequest.withOutput(-(bangBangVolts3 + feedfowardVolts3)).withEnableFOC(false)); //-
    ShooterMotorRev4.setControl(voltageRequest.withOutput(-(bangBangVolts4 + feedfowardVolts4)).withEnableFOC(false)); //-


    SmartDashboard.putNumber("Shooter1Volts", bangBangVolts1 + feedfowardVolts1);
    SmartDashboard.putNumber("Shooter2Volts", bangBangVolts2 + feedfowardVolts2);
    SmartDashboard.putNumber("Shooter3Volts", bangBangVolts3 + feedfowardVolts3);
    SmartDashboard.putNumber("Shooter4Volts", bangBangVolts4 + feedfowardVolts4);

    SmartDashboard.putNumber("Shooter1RPS", currentVelocity1RPS);
    SmartDashboard.putNumber("Shooter2RPS", currentVelocity2RPS);
    SmartDashboard.putNumber("Shooter3RPS", currentVelocity3RPS);
    SmartDashboard.putNumber("Shooter4RPS", currentVelocity4RPS);

    SmartDashboard.putNumber("TargetRPS1", targetVeloRPS1);
    SmartDashboard.putNumber("TargetRPS2", targetVeloRPS2);
    SmartDashboard.putNumber("TargetRPS3", targetVeloRPS3);
    SmartDashboard.putNumber("TargetRPS4", targetVeloRPS4);

    SmartDashboard.putNumber("ShooterMult", ShooterMult);
    SmartDashboard.putBoolean("MaxRPM?", MaxRPM);
    SmartDashboard.putBoolean("MultpilerOn?", MultiplierOn);
  }
}
