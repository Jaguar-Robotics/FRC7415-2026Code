// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.motorcontrollers.SmartMotorController;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


public class ShooterSubsystem extends SubsystemBase {

  
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(1, 0, 0, DegreesPerSecond.of(3600), DegreesPerSecondPerSecond.of(3600))
  .withSimClosedLoopController(0.48, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0.19, 1.13))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 1.13))
  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  // Vendor motor controller object
  private TalonFX shooterLeader = new TalonFX(Constants.ShooterConstants.ShooterLeaderID, "Upper");
  private TalonFX shooterFollower = new TalonFX(Constants.ShooterConstants.ShooterFollowerID, "Upper");
  private TalonFX shooterFollowerReversed = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversedID, "Upper");
  private TalonFX shooterFollowerReversed2 = new TalonFX(Constants.ShooterConstants.ShooterFollowerReversed2ID, "Upper");

  private TalonFXSimState shooterMotorSim = shooterLeader.getSimState();

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController SmartMotorController = new TalonFXWrapper(shooterLeader, DCMotor.getKrakenX60(4), smcConfig);

  
  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(SmartMotorController)
  // Diameter of the flywheel.
  .withDiameter(Inches.of(4))
  // Mass of the flywheel.
  .withMass(Pounds.of(5.2)) //CHangle later maybe
  // Maximum speed of the shooter.
  .withUpperSoftLimit(RPM.of(3200))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel shooter = new FlyWheel(shooterConfig);

  
  AngularVelocity setVelo = RPM.of(0);
   /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return shooter.getSpeed();}

  public Command stop() {
    setVelo = RPM.of(0); 
    return shooter.set(0);}

/**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}
  
  /**
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    if (speed.gte(Constants.ShooterConstants.SetRPMHardStop)){ speed = Constants.ShooterConstants.SetRPMHardStop;}
    setVelo = speed;
    return shooter.setSpeed(speed);}


  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) { return shooter.set(dutyCycle);}

  private CommandSwerveDrivetrain drivetrain;

    public ShooterSubsystem() {

    //-------------COMMENT ALL THIS OUT IF TESTING WITH PHYNEX TUNER-----------------
    shooterFollower.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Aligned));

    // 2. Reversed Followers (Opposite direction of leader)
    shooterFollowerReversed.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Opposed));
    shooterFollowerReversed2.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Opposed));
  }

  /** Creates a new ShooterSubsystem. with drivetrain perameters */
  public ShooterSubsystem(CommandSwerveDrivetrain drivetrin) {
    //-------------COMMENT ALL THIS OUT IF TESTING WITH PHYNEX TUNER-----------------
    shooterFollower.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Aligned));

    // 2. Reversed Followers (Opposite direction of leader)
    // Use 'true' for the opposeLeader parameter
    shooterFollowerReversed.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Opposed));
    shooterFollowerReversed2.setControl(new Follower(Constants.ShooterConstants.ShooterLeaderID, MotorAlignmentValue.Opposed));
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
    
    if (velo.gte(Constants.ShooterConstants.SetRPMHardStop)){ velo = Constants.ShooterConstants.SetRPMHardStop;}
    setVelo = velo; 
    return velo;
}

  public Command setIdleMode(Pose2d RobotPose){
    Pose2d currentPose = RobotPose;
    double robotX = currentPose.getX();
    
    // Define field zones (adjust these values based on your field layout)
    // Assuming field is ~16.5 meters long (54 feet)
    double ourZoneMax = 5.5;      // meters - our third of the field
    double middleZoneMax = 11.0;  // meters - middle third
    // Anything beyond middleZoneMax is "their" zone
    
    // Determine which zone we're in and set appropriate state
    if (robotX < ourZoneMax) {
        return setVelocity(Constants.ShooterConstants.SlowShot);
    } else if (robotX < middleZoneMax) {
        return stop();
    } else {
        return stop();
    }
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
  public void periodic() {

    SmartDashboard.putNumber("Request RPM", setVelo.in(RPM));
    SmartDashboard.putNumber("Real RPM", getVelocity().in(RPM));
    
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }
  
@Override
public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
    //SmartDashboard.putNumber("Request RPM", setVelo.in(RPM));
    //SmartDashboard.putNumber("Real RPM", getVelocity().in(RPM));

    

    //System.out.println("Sim periodic running - Leader voltage: " + shooterLeader.getSimState().getMotorVoltage());
    // Log voltages for all motors
    double leaderVoltage = shooterLeader.getSimState().getMotorVoltage();
    shooterFollower.getSimState().setSupplyVoltage(leaderVoltage);
    shooterFollowerReversed.getSimState().setSupplyVoltage(leaderVoltage);
    shooterFollowerReversed2.getSimState().setSupplyVoltage(leaderVoltage);

    SmartDashboard.putNumber("Shooter/Leader Voltage", shooterLeader.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/Follower Voltage", shooterFollower.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/FollowerReversed Voltage", shooterFollowerReversed.getSimState().getMotorVoltage());
    SmartDashboard.putNumber("Shooter/FollowerReversed2 Voltage", shooterFollowerReversed2.getSimState().getMotorVoltage());
  }
}
