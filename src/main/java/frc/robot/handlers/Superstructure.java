package frc.robot.handlers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.handlers.IndexerHighHandler.IndexerHighState;
import frc.robot.handlers.IndexerLowHandler.IndexerLowState;
import frc.robot.subsystems.BangBangShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerHighSubsystem;
import frc.robot.subsystems.IndexerLowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;

public class Superstructure extends SubsystemBase {

  // Define your states
  public enum SuperstructureState {
    STATIONARYSHOT,
    INTAKE,
    SLOWSHOT,
    REVERSE,
    SPINUP,
    TUNING,
    OFF,
    IDLE,
    SPINUPSLOW,
    SPINUPFAST,
    FASTSHOT,
    AIM
  }

  private static Superstructure instance;
  private BangBangShooterSubsystem shooter; 
  private CommandSwerveDrivetrain drivetrain;
  
  //private final ShooterSubsystem Shooter = new ShooterSubsystem();
  private final ShooterHandler shooterHandler = ShooterHandler.getInstance();

  //private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final IntakeHandler intakeHandler = IntakeHandler.getInstance();

  private final IntakeSlideHandler intakeSlideHandler = IntakeSlideHandler.getInstance();

  //private final HopperSubsystem Hopper = new HopperSubsystem();
  private final HopperHandler hopperHandler = HopperHandler.getInstance();

  //private final IndexerSubsystem Indexer = new IndexerSubsystem();
  private final IndexerHighHandler indexerHighHandler = IndexerHighHandler.getInstance();
  private final IndexerLowHandler indexerLowHandler = IndexerLowHandler.getInstance();

  private final DriveHandler driveHandler = DriveHandler.getInstance();

  private SuperstructureState desiredState = SuperstructureState.IDLE;
  private SuperstructureState currentState = SuperstructureState.IDLE;
  private Angle targetAngle = Degrees.of(0);
  

  public static Superstructure getInstance(){
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

    /**
   * Initialize superstructure with required subsystems
   */
  public void initialize(BangBangShooterSubsystem shooter, CommandSwerveDrivetrain drivetrain) {
      this.shooter = shooter;
      this.drivetrain = drivetrain;
  }

  private Superstructure() {}

  /** Call this from commands or joystick logic to set the next goal. */
  public void setDesiredState(SuperstructureState newState) {
    System.out.print("SetDesiredState:" + newState);
    if (desiredState != newState) {
      desiredState = newState;
      handleStateTransition();
    }
  }

  /** Handles transitions between states (2910-style logic). */
  private void handleStateTransition() {
    switch (desiredState) {
      case IDLE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.BRAKE);
        break;
      case INTAKE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.OUT);
        break;
      case SPINUP:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING); //change to shooting
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case STATIONARYSHOT:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SHOOTING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);
        break;
      case TUNING: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.TUNING);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        break;
      case SLOWSHOT: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.SLOW);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.SLOW);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.SLOWINTAKE);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.SLOWINTAKE);
        //driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case SPINUPFAST: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.TUNING); //SWITCHEWD TO TUNING SWITCH BACK TO FAST SHOT
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerHighHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
         break;
      case FASTSHOT: //dont use
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.TUNING); //SWITCHED TO TUNABLE FAST SHOT SWITCH BACK TO FAST
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.SLOWINTAKE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.FAST);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.FAST);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.FAST);
        //driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        //intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.SLOWIN);
        break;
      case REVERSE:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.FASTREVERSE);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        break;
      case AIM:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.AUTOALLIGN);
        break;
      case OFF:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.IN);
        break;
      default:
        shooterHandler.setDesiredState(ShooterHandler.ShooterState.OFF);
        intakeHandler.setDesiredState(IntakeHandler.IntakeState.OFF);
        hopperHandler.setDesiredState(HopperHandler.HopperState.OFF);
        indexerHighHandler.setDesiredState(IndexerHighHandler.IndexerHighState.OFF);
        indexerLowHandler.setDesiredState(IndexerLowHandler.IndexerLowState.OFF);
        driveHandler.setDesiredState(DriveHandler.DriveState.TELEOPDRIVE);
        intakeSlideHandler.setDesiredState(IntakeSlideHandler.IntakeSlideState.IN);
        break;
    }
    currentState = desiredState;
  }

    boolean DTaimed = false;
  @Override
  public void periodic() {

        if (shooter == null) {
        System.err.println("ERROR: Superstructure not initialized!");
        return;
        }
        
        DTaimed = drivetrain.isAimedAtTarget();

        if (currentState == SuperstructureState.SPINUP && (shooter.atTargetVelo() && DTaimed)){ //checks if its at target velo and angle
          setDesiredState(SuperstructureState.STATIONARYSHOT);
        }

        SmartDashboard.putString("SuperState", currentState.toString());

        SmartDashboard.putBoolean("shooterAtVelo?", shooter.atTargetVelo());
        SmartDashboard.putBoolean("Drivetrain aimed?",DTaimed);
         
  }
  public SuperstructureState getCurrentState() {
    return currentState;
  }

  public SuperstructureState getDesiredState() {
    return desiredState;
  }
}