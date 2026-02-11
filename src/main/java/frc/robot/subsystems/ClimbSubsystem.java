package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simple climb subsystem using TalonFX for telescoping climb mechanism.
 * Does NOT have an absolute encoder - uses "self-homing" command to reset position.
 */

 //ndsb zcjndcbsdjkbhads

 //dcads bhvadjk vsedwue jbsfr
 //adjk bahdc vads
 //now that youve tnoticed my ahh spam, fix climb later, remember its permanently out
 //and you have to bring the climb back, so do negative distance on motor
 //when you can
 
public class ClimbSubsystem extends SubsystemBase {
  
  // Hardware
  private final TalonFX climbMotor = new TalonFX(Constants.ClimberConstants.ClimberMotorID);
  
  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  
  // Mechanism Configuration
  private final double gearRatio = 12.0; // CHECK THIS VALUE BRU
  private final double spocketCircumferenceMeters = 0.25 * 0.0254 * 22; // ALSO CHECK BRU I WAS JS TYPIN SHI https://tenor.com/view/bro-js-typing-shit-type-shit-bro-roblox-nagi-gracjan-gif-6356208401846398604
  private final double rotationsPerMeter = gearRatio / spocketCircumferenceMeters;
  
  // Position Limits
  private final double softLowerLimitMeters = 0.0;
  private final double softUpperLimitMeters = 2.0; 
  private final double hardLowerLimitMeters = 0.0;
  private final double hardUpperLimitMeters = 3.0;
  
  // Control
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, .5, 0, 0); // Tune kG for gravity compensation
  private final ProfiledPIDController pidController = new ProfiledPIDController(
      10.0, 0, 0, // Tune these PID values cro
      new TrapezoidProfile.Constraints(1.0, 2.0) // max velocity, max accel in m/s
  );
  
  private double targetHeightMeters = 0.0;

  public ClimbSubsystem() {
    // Configure TalonFX
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softUpperLimitMeters * rotationsPerMeter;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLowerLimitMeters * rotationsPerMeter;
    
    climbMotor.getConfigurator().apply(config);
    climbMotor.setPosition(0); // Start at zero, must home first!
  }

  @Override
  public void periodic() {
    // Update PID control if we have a target
    if (pidController.getGoal().position != targetHeightMeters) {
      pidController.setGoal(targetHeightMeters);
    }
    
    double currentHeightMeters = climbMotor.getPosition().getValueAsDouble() / rotationsPerMeter;
    double pidOutput = pidController.calculate(currentHeightMeters);
    double ffOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    
    climbMotor.setControl(voltageRequest.withOutput(pidOutput + ffOutput));
  }

  /**
   * Homes the climb by running it down until current threshold is exceeded.
   * 
   * @param thresholdAmps The current threshold that indicates hitting the hard limit
   * @return Command that homes the climb mechanism
   */


   //Runs the climb down until it hits the hard stop (detected by current spike), then zeros the encoder
  public Command homing(double thresholdAmps) { 
    Debouncer currentDebouncer = new Debouncer(0.4);
    double homingVoltage = -2.0;
    double velocityThreshold = 0.1; // rotations per second
    
    return Commands.runOnce(() -> {
      pidController.reset(0); // Stop PID
      targetHeightMeters = 0;
    })
    .andThen(Commands.run(() -> {
      climbMotor.setControl(voltageRequest.withOutput(homingVoltage));
    }))
    .until(() -> currentDebouncer.calculate(
        climbMotor.getStatorCurrent().getValueAsDouble() >= thresholdAmps &&
        Math.abs(climbMotor.getVelocity().getValueAsDouble()) <= velocityThreshold
    ))
    .finallyDo(() -> {
      climbMotor.setPosition(0); // Reset encoder to zero at bottom
      targetHeightMeters = 0;
      pidController.reset(0);
    });
  }

  /**
   * Runs the climb at a specified duty cycle (percent output).
   * 
   * @param dutyCycle Duty cycle from -1.0 to 1.0
   * @return Command to run the climb
   */
  public Command climbCmd(double dutyCycle) {
    return Commands.runOnce(() -> {
      targetHeightMeters = getCurrentHeight(); // Hold current position
      pidController.reset(getCurrentHeight());
    }).andThen(Commands.run(() -> {
      climbMotor.setControl(voltageRequest.withOutput(dutyCycle * 12.0));
    }));
  }

  /**
   * Sets the climb to a specific height using closed-loop control.
   * 
   * @param height Target height in meters
   * @return Command to move climb to specified height
   */
  public Command setHeight(Distance height) {
    return Commands.runOnce(() -> {
      targetHeightMeters = height.in(Meters);
    });
  }

  /**
   * Gets the current height of the climb.
   * 
   * @return Current height in meters
   */
  public double getCurrentHeight() {
    return climbMotor.getPosition().getValueAsDouble() / rotationsPerMeter;
  }
  
  /**
   * Checks if the climb is at the target height within tolerance.
   * 
   * @return true if at target height
   */
  public boolean atTarget() {
    return pidController.atGoal();
  }
  
  /**
   * Stops the climb motor.
   * 
   * @return Command to stop the climb
   */
  public Command stop() {
    return Commands.runOnce(() -> {
      targetHeightMeters = getCurrentHeight();
      climbMotor.setControl(voltageRequest.withOutput(0));
    });
  }
}