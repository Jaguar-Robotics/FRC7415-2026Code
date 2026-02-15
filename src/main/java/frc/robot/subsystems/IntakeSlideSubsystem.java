// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSlideSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
    .withMechanismCircumference(Meters.of(Inches.of(Math.PI).in(Meters)))
    // Feedback Constants (PID Constants)
    .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    // Feedforward Constants
    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    // Telemetry name and verbosity level
    .withTelemetry("IntakeSlide", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(0.5)))
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25));

  // Vendor motor controller object
  private TalonFX IntakeSlideMotor = new TalonFX(Constants.IntakeConstants.IntakeSlideMotorID);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController TalonSmartMotorController = new TalonFXWrapper(IntakeSlideMotor, DCMotor.getKrakenX60(1), smcConfig);

  private ElevatorConfig elevconfig = new ElevatorConfig(TalonSmartMotorController)
      .withStartingHeight(Meters.of(0))
      .withHardLimits(Meters.of(0), Inches.of(13.3))
      .withTelemetry("IntakeSlide", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(12));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return elevator.run(height);}
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to., tolerancce.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) { return elevator.runTo(height, Inches.of(1));}
  
  /**
   * Set the elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return elevator.set(dutycycle);}

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  public boolean isCurrentSpike(){
    IntakeSlideMotor.getStatorCurrent().refresh();
    double currentCurrent = IntakeSlideMotor.getStatorCurrent().getValueAsDouble();
    double THRESHOLD = 40.0;
    return currentCurrent > THRESHOLD;
  }

  public Command runUntilStall(double power) {
    return Commands.run(() -> this.set(power), this)
        .until(this::isCurrentSpike)
        .withTimeout(5.0);
}

  /** Creates a new ExampleSubsystem. */
  public IntakeSlideSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
