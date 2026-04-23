// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.handlers;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.handlers.StateSubsystem.State;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveHandler extends SubsystemBase {

    public enum DriveState implements State {
        TELEOPDRIVE,
        TELEOPDRIVESLOW,
        TOWERDRIVE,
        AUTOALLIGN,
        SHOOTONTHEMOVE,
        SNAKE,
        XDRIVE,
        BUMP_LOCK
  }


  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController joystick;
  private SwerveRequest.FieldCentric drive;
  private double maxSpeed;
  private double maxAngularRate;
  private static DriveHandler instance;



  private DriveState desiredState = DriveState.TELEOPDRIVE; 
  private DriveState currentState = DriveState.TELEOPDRIVE;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** Creates a new IntakeHandler. */
  private DriveHandler() {}

  public static DriveHandler getInstance(){
    if (instance == null){
      instance = new DriveHandler();
    }
    return instance;
  }

  Trigger inTower = new Trigger(() -> {
      Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
      return Constants.FieldConstants.RedTower.contains(robotPos) ||
            Constants.FieldConstants.BlueTower.contains(robotPos);
  });


  public void initialize(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, SwerveRequest.FieldCentric drive,  double maxSpeed,  double maxAngularRate) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    this.drive = drive;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;

    update();
  }

  
  public void setDesiredState(DriveState state){
        if (desiredState != state) {
        desiredState = state;
        updateStates();
    }
  }

  public void updateStates(){
    if (currentState != desiredState) {
        currentState = desiredState;
        update();
    }
  }

  public void handleStateTransition() {
    update();
  }

    public void update() {

    Command currentCmd = drivetrain.getCurrentCommand(); //removes current command
    if (currentCmd != null) {
        CommandScheduler.getInstance().cancel(currentCmd);
    }
        switch (desiredState) {
            case TELEOPDRIVE:
                drivetrain.setDefaultCommand(drivetrain.TeleopDrive(joystick, maxSpeed, maxAngularRate, drive, drivetrain));
                break;
            case TELEOPDRIVESLOW:
                drivetrain.setDefaultCommand(drivetrain.TeleopDriveSLOW(joystick, maxSpeed, maxAngularRate, drive, drivetrain));
                break;
            case TOWERDRIVE:
                drivetrain.setDefaultCommand(drivetrain.TeleopDrive(joystick, maxSpeed/3, maxAngularRate/3, drive, drivetrain));
                break;
            case AUTOALLIGN:
                drivetrain.setDefaultCommand(drivetrain.headingLocktoHub(joystick, maxSpeed, maxAngularRate));
                break;
           case SHOOTONTHEMOVE:
                drivetrain.setDefaultCommand(drivetrain.shootOnTheMoveIterative(joystick, maxSpeed, maxAngularRate));
               break;
            case SNAKE:
                drivetrain.setDefaultCommand(drivetrain.getSnakeDriveCommand(drive, drivetrain, joystick, maxSpeed, maxAngularRate));
                break;
            case XDRIVE:
                drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> brake));
                break;
            case BUMP_LOCK:
                drivetrain.setDefaultCommand(drivetrain.bumpLockCommand(drive, drivetrain, joystick, maxSpeed, maxAngularRate)); 
                break;
            default:
                drivetrain.setDefaultCommand(drivetrain.TeleopDrive(joystick, maxSpeed, maxAngularRate, drive, drivetrain));
                break;
        }
        currentState = desiredState;
    }

      public DriveState getCurrentState() {
      return currentState;
  }

  @Override
  public void periodic() {
    if ((currentState.equals(DriveState.TELEOPDRIVE) || currentState.equals(DriveState.TELEOPDRIVESLOW)) && inTower.getAsBoolean()){
      setDesiredState(DriveState.TOWERDRIVE);
    }
    if (currentState.equals(DriveState.TOWERDRIVE) && !inTower.getAsBoolean()){
      setDesiredState(DriveState.TELEOPDRIVESLOW);
    }
    SmartDashboard.putString("DriveState", currentState.toString());
    SmartDashboard.putString("Current Command", drivetrain.getCurrentCommand() != null ? drivetrain.getCurrentCommand().getName() : "null");
    // This method will be called once per scheduler run
  }
}

