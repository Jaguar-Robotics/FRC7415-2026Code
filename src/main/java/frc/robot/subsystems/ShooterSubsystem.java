// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.handlers.IntakeHandler;
import frc.robot.handlers.Superstructure.SuperstructureState;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


public class ShooterSubsystem extends SubsystemBase {

  // motors
  private TalonFX shooterLeader = new TalonFX(Constants.ShooterConstants.ShooterLeaderID);
  private TalonFX shooterFollower1 = new TalonFX(Constants.ShooterConstants.ShooterFollowerID);
  private TalonFX shooterFollower2 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversedID);
  private TalonFX shooterFollower3 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversed2ID);

  private TalonFXSimState shooterMotorSim = shooterLeader.getSimState();

  private double f = 0.4;
  
  AngularVelocity setVelo = RPM.of(0);

  private CommandSwerveDrivetrain drivetrain;


  // Creates a BangBangController
  BangBangController controller = new BangBangController();

  public ShooterSubsystem(){
    shooterFollower1.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Aligned));
    shooterFollower2.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Aligned));
    shooterFollower3.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Aligned));
  }

  private static ShooterSubsystem instance;
  public static ShooterSubsystem getInstance(){
      if (instance == null){
          instance = new ShooterSubsystem();
      }
      return instance;
  }

  public void initialize(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Get current velocity from the motor
  public AngularVelocity getVelocity() {
    // TalonFX getVelocity() returns rotations per second, convert to RPM
    double rps = shooterLeader.getVelocity().getValueAsDouble(); 
    return RPM.of(rps * 60.0);
  }

  public Command setVelocity(AngularVelocity speed) {
    return Commands.runOnce(() -> {
        AngularVelocity finalSpeed = speed;
        if (speed.gte(Constants.ShooterConstants.SetRPMHardStop)) {
            finalSpeed = Constants.ShooterConstants.SetRPMHardStop;
        }
        setVelo = finalSpeed;
    }, this);
}

  // Stop the shooter
  public Command stop() {
    return Commands.runOnce(() -> {
        setVelo = RPM.of(0);
        shooterLeader.set(0);
    }, this);
}


  //Checking if shooter RPM is at threashold (waiting for it to spinup)
  public boolean isAtTargetVelo(){
    double currentRPM = getVelocity().in(RPM);
    double targetRPM = setVelo.in(RPM);

    SmartDashboard.putBoolean("Shooter/At Target", currentRPM >= targetRPM);

    return currentRPM >= targetRPM;
  }

  public AngularVelocity getCalcedRPM(double DistMeters){
    
    //double distanceMeters = drivetrain.getDistance();
    double distanceInches = DistMeters * 39.3701;
    AngularVelocity velo = RPM.of(-0.0312466 * Math.pow(distanceInches, 2) + 29.07009 * distanceInches + 828.29202);
    
    if (velo.gte(Constants.ShooterConstants.SetRPMHardStop)) {
        velo = Constants.ShooterConstants.SetRPMHardStop;
    }
    
    return velo;
}

  public boolean isReadyToShoot() {
    // Check if at target velocity and drivetrain is aimed
    return isAtTargetVelo() && drivetrain != null && isAimedAtTarget();
  }

  public boolean isAimedAtTarget() {
    if (drivetrain == null) return false;
    
    Pose2d robotPose = drivetrain.getPose();
    Pose2d targetPose = CommandSwerveDrivetrain.getHubPose().toPose2d();
    
    // Calculate angle error
    Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());
    Rotation2d targetAngle = toTarget.getAngle().rotateBy(Rotation2d.k180deg);
    Rotation2d currentAngle = robotPose.getRotation();
    
    double errorDegrees = Math.abs(targetAngle.minus(currentAngle).getDegrees());
    
    SmartDashboard.putNumber("Shooter/Aim Error (deg)", errorDegrees);
    
    return errorDegrees < 2.0; // Within 2 degrees
  }

  @Override
  public void periodic(){
    double output = controller.calculate(getVelocity().in(RPM), setVelo.in(RPM));
    shooterLeader.set(output);
    
    SmartDashboard.putNumber("Request RPM", setVelo.in(RPM));
    SmartDashboard.putNumber("Real RPM", getVelocity().in(RPM));
    SmartDashboard.putNumber("Bang-Bang Output", output);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Log voltages for all motors
    double leaderVoltage = shooterLeader.getSimState().getMotorVoltage();
    shooterFollower1.getSimState().setSupplyVoltage(leaderVoltage);
    shooterFollower2.getSimState().setSupplyVoltage(leaderVoltage);
    shooterFollower3.getSimState().setSupplyVoltage(leaderVoltage);

    SmartDashboard.putNumber("Shooter/Leader Voltage", shooterLeader.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/Follower Voltage", shooterFollower1.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/FollowerReversed Voltage", shooterFollower2.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/FollowerReversed2 Voltage", shooterFollower3.getSimState().getMotorVoltage());
  }
}